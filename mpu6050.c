#include "MPU6050.h"

#define ACCEL_XOFFS_H 0x06
#define ACCEL_XOFFS_L 0x07
#define ACCEL_YOFFS_H 0x08
#define ACCEL_YOFFS_L 0x09
#define ACCEL_ZOFFS_H 0x0A
#define ACCEL_ZOFFS_L 0x0B
#define GYRO_XOFFS_H 0x13
#define GYRO_XOFFS_L 0x14
#define GYRO_YOFFS_H 0x15
#define GYRO_YOFFS_L 0x16
#define GYRO_ZOFFS_H 0x17
#define GYRO_ZOFFS_L 0x18
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B  // Gyroscope Configuration
#define ACCEL_CONFIG 0x1C // Accelerometer Configuration
#define FF_THRESHOLD 0x1D
#define FF_DURATION 0x1E
#define MOT_THRESHOLD 0x1F
#define MOT_DURATION 0x20
#define ZMOT_THRESHOLD 0x21
#define ZMOT_DURATION 0x22
#define INT_PIN_CFG 0x37 // INT Pin. Bypass Enable Configuration
#define INT_ENABLE 0x38  // INT Enable
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define MOT_DETECT_STATUS 0x61
#define MOT_DETECT_CTRL 0x69
#define USER_CTRL 0x6A  // User Control
#define PWR_MGMT_1 0x6B // Power Management 1
#define WHO_AM_I 0x75   // Who Am I

#define SMPLRT_DIV 0x19

#define GRAVITY_CONSTANT 9.80665f


bool mpu6050_init(struct mpu6050 *self, i2c_inst_t *i2c_instance) {

  self->i2c.instance = i2c_instance;
  self->i2c.address = MPU6050_ADDRESS;

  if(mpu6050_connected(self) == 0) {
    return false;
  }

  // set clock config to PLL with Gyro X reference
  mpu6050_set_clock_source(self, MPU6050_CLOCK_PLL_XGYRO); // Default Clock

  // Set Tasa de muestreo
  mpu6050_set_sample_rate(self, 0);

  // set accelerometer range to +-2G
  mpu6050_set_range(self, MPU6050_RANGE_2G);              // Default Range

  // set gyro range to +- 250 deg/s
  mpu6050_set_scale(self, MPU6050_SCALE_250DPS);          // Default Scale

  mpu6050_set_sleep_enabled(self, 0);                     // Disable Sleep Mode

  // set filter bandwidth to 94 Hz
  mpu6050_set_dlpf_mode(self, MPU6050_DLPF_2); //94Hz

  self->gyro_offsets.x = 0;
  self->gyro_offsets.y = 0;
  self->gyro_offsets.z = 0;

  self->accel_offsets.x = 0;
  self->accel_offsets.y = 0;
  self->accel_offsets.z = 0;

  self->calibrated = false;

  return true;
}

uint8_t mpu6050_connected(struct mpu6050 *self)
{
    if (mpu6050_who_am_i(self) != 0x68) // 0x68 default WHO_AM_I value
    {
        return 0;
    }

    return 1;
}

uint8_t mpu6050_who_am_i(struct mpu6050 *self)
{
    uint8_t who_am_i;
    i2c_read_reg(self->i2c.instance, self->i2c.address, WHO_AM_I, &who_am_i, 1);
    return who_am_i;
}


// LECTURA
void mpu6050_read_raw_gyro(struct mpu6050 *self, int16_t *gx, int16_t *gy, int16_t *gz) {
  uint8_t data[6];
  i2c_read_reg(self->i2c.instance, self->i2c.address, GYRO_XOUT_H, data, 6);

  *gx = data[0] << 8 | data[1];
  *gy = data[2] << 8 | data[3];
  *gz = data[4] << 8 | data[5];
}

void mpu6050_read_raw_accel(struct mpu6050 *self, int16_t *ax, int16_t *ay, int16_t *az) {
  uint8_t data[6];
  i2c_read_reg(self->i2c.instance, self->i2c.address, ACCEL_XOUT_H, data, 6);

  *ax = data[0] << 8 | data[1];
  *ay = data[2] << 8 | data[3];
  *az = data[4] << 8 | data[5];
}

void mpu6050_read(struct mpu6050 *self, struct float_vector *gyro, struct float_vector *accel) {
  int16_t gx, gy, gz;
  mpu6050_read_raw_gyro(self, &gx, &gy, &gz);

  int16_t ax, ay, az;
  mpu6050_read_raw_accel(self, &ax, &ay, &az);

  if(self->calibrated) {
    gyro->x = (gx / self->config.dps_per_digit) - self->gyro_offsets.x;
    gyro->y = (gy / self->config.dps_per_digit) - self->gyro_offsets.y;
    gyro->z = (gz / self->config.dps_per_digit) - self->gyro_offsets.z;

    accel->x = (ax * self->config.range_per_digit * GRAVITY_CONSTANT) + self->accel_offsets.x;
    accel->y = (ay * self->config.range_per_digit * GRAVITY_CONSTANT) + self->accel_offsets.y;
    accel->z = (az * self->config.range_per_digit * GRAVITY_CONSTANT) + self->accel_offsets.z;
  }
  else {
    gyro->x = (gx / self->config.dps_per_digit);
    gyro->y = (gy / self->config.dps_per_digit);
    gyro->z = (gz / self->config.dps_per_digit);

    accel->x = (ax * self->config.range_per_digit * GRAVITY_CONSTANT);
    accel->y = (ay * self->config.range_per_digit * GRAVITY_CONSTANT);
    accel->z = (az * self->config.range_per_digit * GRAVITY_CONSTANT);
  }
}



// CALIBRACION / OFFSETS
void mpu6050_set_gyro_offsets(mpu6050_t *self, float gyro_offsets[3]){
  self->gyro_offsets.x = gyro_offsets[0];
  self->gyro_offsets.y = gyro_offsets[1];
  self->gyro_offsets.z = gyro_offsets[2];
}

void mpu6050_set_accel_offsets(mpu6050_t *self, float_vector_t *accel_offsets){
  self->accel_offsets = *accel_offsets;

}

// void mpu6050_set_accel_offsets(mpu6050_t *self, float accel_offsets[3]){
  // self->accel_offsets.x = accel_offsets[0];
  // self->accel_offsets.y = accel_offsets[1];
  // self->accel_offsets.z = accel_offsets[2];
// }

void mpu6050_get_gyro_offsets(mpu6050_t *self, float gyro_offsets[3]) {
  gyro_offsets[0] = self->gyro_offsets.x;
  gyro_offsets[1] = self->gyro_offsets.y;
  gyro_offsets[2] = self->gyro_offsets.z;
}

void mpu6050_get_accel_offsets(mpu6050_t *self, float_vector_t *accel_offsets) {
  *accel_offsets = self->accel_offsets;
}

// void mpu6050_get_accel_offsets(mpu6050_t *self, float accel_offsets[3]){
  // accel_offsets[0] = self->accel_offsets.x;
  // accel_offsets[1] = self->accel_offsets.y;
  // accel_offsets[2] = self->accel_offsets.z;
// }

// SETTERS PARA CONFIGURACION
void mpu6050_set_clock_source(struct mpu6050 *self, enum MPU6050_CLOCK_SOURCE clock_source)
{
    uint8_t power_managment_1;
    i2c_read_reg(self->i2c.instance, self->i2c.address, PWR_MGMT_1, &power_managment_1, 1);
    power_managment_1 &= 0xF8;
    power_managment_1 |= clock_source;

    uint8_t data[2] = {PWR_MGMT_1, power_managment_1};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_sample_rate(struct mpu6050 *self, uint8_t sample_rate_divisor)
{
    uint8_t sample_rate_value = 0;

    // Validar que el divisor de tasa de muestreo esté en el rango permitido (0 a 255)
    if (sample_rate_divisor > 255)
    {
        sample_rate_divisor = 255;  // Limitar a 255 si se pasa de 255
    }

    // Configurar la tasa de muestreo en el registro SMPLRT_DIV
    sample_rate_value = sample_rate_divisor;

    uint8_t data[2] = {SMPLRT_DIV, sample_rate_value};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_range(struct mpu6050 *self, enum MPU6050_RANGE range)
{
    uint8_t accel_config;

    switch (range)
    {
    case MPU6050_RANGE_2G:
        self->config.range_per_digit = .000061f;
        break;
    case MPU6050_RANGE_4G:
        self->config.range_per_digit = .000122f;
        break;
    case MPU6050_RANGE_8G:
        self->config.range_per_digit = .000244f;
        break;
    case MPU6050_RANGE_16G:
        self->config.range_per_digit = .0004882f;
        break;
    }

    i2c_read_reg(self->i2c.instance, self->i2c.address, ACCEL_CONFIG, &accel_config, 1);
    accel_config &= 0xE7;
    accel_config |= (range << 3);

    uint8_t data[2] = {ACCEL_CONFIG, accel_config};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_scale(struct mpu6050 *self, enum MPU6050_SCALE scale)
{
    uint8_t gyro_config;

    switch (scale)
    {
    case MPU6050_SCALE_250DPS:
        self->config.dps_per_digit = 131.0;
        break;
    case MPU6050_SCALE_500DPS:
        self->config.dps_per_digit = 65.5;
        break;
    case MPU6050_SCALE_1000DPS:
        self->config.dps_per_digit = 32.8;
        break;
    case MPU6050_SCALE_2000DPS:
        self->config.dps_per_digit = 16.4;
        break;
    }

    i2c_read_reg(self->i2c.instance, self->i2c.address, GYRO_CONFIG, &gyro_config, 1);
    gyro_config &= 0xE7;
    gyro_config |= (scale << 3);

    uint8_t data[2] = {GYRO_CONFIG, gyro_config};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_sleep_enabled(struct mpu6050 *self, uint8_t state)
{
    uint8_t data[2] = {PWR_MGMT_1};
    if (state)
    {
        data[1] = 1;
    }
    else
    {
        data[1] = 0;
    }

    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_dlpf_mode(struct mpu6050 *self, enum MPU6050_DLPF dlpf)
{
    uint8_t value;
    i2c_read_reg(self->i2c.instance, self->i2c.address, CONFIG, &value, 1);
    value &= 0xF8;
    value |= dlpf;
    uint8_t data[2] = {CONFIG, value};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void mpu6050_set_dhpf_mode(struct mpu6050 *self, enum MPU6050_DHPF dhpf)
{
    uint8_t value;
    i2c_read_reg(self->i2c.instance, self->i2c.address, ACCEL_CONFIG, &value, 1);
    value &= 0xF8;
    value |= dhpf;
    uint8_t data[2] = {ACCEL_CONFIG, value};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}
