#ifndef MPU6050_H_
#define MPU6050_H_

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "config.h"
#include "I2C.h"



#ifdef __cplusplus
extern "C"
{
#endif



#define MPU6050_ADDRESS_A0_VCC 0x69
#define MPU6050_ADDRESS_A0_GND 0x68

#define MPU6050_ADDRESS 0x68


enum MPU6050_CLOCK_SOURCE
{
    MPU6050_CLOCK_INTERNAL = 0,
    MPU6050_CLOCK_PLL_XGYRO = 1,
    MPU6050_CLOCK_PLL_YGYRO = 2,
    MPU6050_CLOCK_PLL_ZGYRO = 3,
    MPU6050_CLOCK_EXTERNAL_32KHZ = 4,
    MPU6050_CLOCK_EXTERNAL_19MHZ = 5,
    MPU6050_CLOCK_KEEP_RESET = 7,
};

enum MPU6050_SCALE
{
    MPU6050_SCALE_250DPS = 0,
    MPU6050_SCALE_500DPS = 1,
    MPU6050_SCALE_1000DPS = 2,
    MPU6050_SCALE_2000DPS = 3
};

enum MPU6050_RANGE
{
    MPU6050_RANGE_2G = 0,
    MPU6050_RANGE_4G = 1,
    MPU6050_RANGE_8G = 2,
    MPU6050_RANGE_16G = 3,
};

enum MPU6050_DHPF
{
    MPU6050_DHPF_RESET = 0,
    MPU6050_DHPF_5HZ = 1,
    MPU6050_DHPF_2_5HZ = 2,
    MPU6050_DHPF_1_25HZ = 3,
    MPU6050_DHPF_0_63HZ = 4,
    MPU6050_DHPF_HOLD = 7
};

enum MPU6050_DLPF
{
    MPU6050_DLPF_0 = 0,
    MPU6050_DLPF_1 = 1,
    MPU6050_DLPF_2 = 2,
    MPU6050_DLPF_3 = 3,
    MPU6050_DLPF_4 = 4,
    MPU6050_DLPF_5 = 5,
    MPU6050_DLPF_6 = 6,
};


struct mpu6050_configuration
{
    float dps_per_digit;
    float range_per_digit;
    uint8_t dhpf;
    uint8_t dlpf;
};

typedef struct mpu6050
{
    struct i2c_information i2c;
    struct mpu6050_configuration config;
    struct float_vector accel_offsets;
    struct float_vector gyro_offsets;
    bool calibrated;
} mpu6050_t;


// INICIALIZACION y CONEXION
bool mpu6050_init(struct mpu6050 *self, i2c_inst_t *i2c_instance);

uint8_t mpu6050_connected(struct mpu6050 *self);

uint8_t mpu6050_who_am_i(struct mpu6050 *self);
// INICIALIZACION y CONEXION


// LECTURA
void mpu6050_read_raw_accel(struct mpu6050 *self, int16_t *ax, int16_t *ay, int16_t *az);

void mpu6050_read_raw_gyro(struct mpu6050 *self, int16_t *gx, int16_t *gy, int16_t *gz);

void mpu6050_read(struct mpu6050 *self, float_vector_t *gyro, float_vector_t *accel);
// LECTURA


// CALIBRACION / OFFSETS
void mpu6050_set_gyro_offsets(mpu6050_t *self, float *gyro_offsets);

void mpu6050_set_accel_offsets(mpu6050_t *self, float *accel_offsets);

void mpu6050_get_gyro_offsets(mpu6050_t *self, float *gyro_offsets);

void mpu6050_get_accel_offsets(mpu6050_t *self, float *accel_offsets);
// CALIBRACION / OFFSETS


// SETTERS PARA CONFIGURACION
void mpu6050_set_clock_source(struct mpu6050 *self, enum MPU6050_CLOCK_SOURCE clock_source);

void mpu6050_set_sample_rate(struct mpu6050 *self, uint8_t sample_rate_divisor);

void mpu6050_set_range(struct mpu6050 *self, enum MPU6050_RANGE range);

void mpu6050_set_scale(struct mpu6050 *self, enum MPU6050_SCALE scale);

void mpu6050_set_sleep_enabled(struct mpu6050 *self, uint8_t state);

void mpu6050_set_dlpf_mode(struct mpu6050 *self, enum MPU6050_DLPF dlpf);

void mpu6050_set_dhpf_mode(struct mpu6050 *self, enum MPU6050_DHPF dhpf);
// SETTERS PARA CONFIGURACION



#ifdef __cplusplus
}
#endif
#endif
