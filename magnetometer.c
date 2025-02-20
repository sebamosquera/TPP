#include "magnetometer.h"

bool magnetometer_init(magnetometer_t* self, i2c_inst_t *i2c_instance) {

  self->i2c.instance = i2c_instance;
  self->i2c.address = HMC5883L_ADDRESS;

  if (!magnetometer_connected(self)) {
      return false; // Evita errores si se pasa un puntero nulo
  }

  magnetometer_set_averaging(self, HMC5883L_SAMPLES_1);

  magnetometer_set_rate(self, HMC5883L_DATARATE_15_HZ);

  magnetometer_set_range(self, HMC5883L_RANGE_1_3GA); // defaullt

  magnetometer_set_mode(self, HMC5883L_MODE_SINGLE);

  self->magnet_offsets.x = 0;
  self->magnet_offsets.y = 0;
  self->magnet_offsets.z = 0;

  self->calibrated = false;

  return true;
}

bool magnetometer_connected(magnetometer_t* self) {

  uint8_t reg = HMC5883L_REG_ID_A;
  i2c_write_blocking(self->i2c.instance, self->i2c.address, &reg, 1, true);

  uint8_t buffer[3];
  if(i2c_read_blocking(self->i2c.instance, self->i2c.address, buffer, 3, false) == 3) {
    return (buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3');
  }

  return false;
}

bool magnetometer_read_raw(magnetometer_t* self, int16_t *x, int16_t *y, int16_t *z) {

  //  Dirección del registro de datos X
  uint8_t reg = HMC5883L_REG_DATAX_H;

  uint8_t buffer[6];
  i2c_read_reg(self->i2c.instance, self->i2c.address, reg, buffer, 6);

  // Convertir los datos de los 6 bytes a enteros de 16 bits
  *x = (((int16_t)buffer[0]) << 8) | buffer[1];  // Eje X
  *z = (((int16_t)buffer[2]) << 8) | buffer[3];  // Eje Z
  *y = (((int16_t)buffer[4]) << 8) | buffer[5];  // Eje Y

  return true;
}

bool magnetometer_read(magnetometer_t* self, int16_vector_t *magnet) {

  int16_t x_raw;
  int16_t y_raw;
  int16_t z_raw;

  if(!magnetometer_read_raw(self, &x_raw, &y_raw, &z_raw)) {
    return false;
  }

  if(self->calibrated) {
    magnet->x = x_raw - self->magnet_offsets.x;
    magnet->y = y_raw - self->magnet_offsets.y;
    magnet->z = z_raw - self->magnet_offsets.z;
  }
  else {
    magnet->x = x_raw;
    magnet->y = y_raw;
    magnet->z = z_raw;
  }

  // En el modo único, se requiere activar explícitamente una nueva medición
  // después de leer los datos, lo que es más eficiente en términos de energía
  // si no necesitas datos continuamente
  if(self->mode == HMC5883L_MODE_SINGLE) {
    magnetometer_set_mode(self, HMC5883L_MODE_SINGLE);
  }

  return true;
}


void magnetometer_set_magnet_offsets(magnetometer_t *self, int16_t magnet_offsets[3]){
  self->magnet_offsets.x = magnet_offsets[0];
  self->magnet_offsets.y = magnet_offsets[1];
  self->magnet_offsets.z = magnet_offsets[2];
}

void magnetometer_get_magnet_offsets(magnetometer_t *self, int16_t magnet_offsets[3]) {
  magnet_offsets[0] = self->magnet_offsets.x;
  magnet_offsets[1] = self->magnet_offsets.y;
  magnet_offsets[2] = self->magnet_offsets.z;
}


// CONFIGURACION DE REGISTROS
void magnetometer_set_averaging(magnetometer_t* self, enum MAGNETOMETRO_AVERAGING averaging)
{
    uint8_t reg_value;

    i2c_read_reg(self->i2c.instance, self->i2c.address, HMC5883L_REG_CONFIG_A, &reg_value, 1);
    // Limpiar bits de promedio (bits [7:5]) y establecer el nuevo valor
    reg_value &= 0x9F;
    reg_value |= (averaging << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1));

    uint8_t data[2] = {HMC5883L_REG_CONFIG_A, reg_value};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void magnetometer_set_rate(magnetometer_t *self, enum MAGNETOMETRO_RATE rate)
{
    uint8_t reg_value;

    // Leer el registro de configuración A (Tasa de muestreo)
    i2c_read_reg(self->i2c.instance, self->i2c.address, HMC5883L_REG_CONFIG_A, &reg_value, 1); // Registro de Configuración A
    reg_value &= 0xE3;  // Limpiar bits de tasa (bits [4:2])
    reg_value |= (rate << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)); // Establecer los nuevos bits de tasa

    // Escribir de vuelta en el registro de Configuración A
    uint8_t data[2] = {HMC5883L_REG_CONFIG_A, reg_value};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void magnetometer_set_bias(magnetometer_t *self, enum MAGNETOMETRO_BIAS bias)
{
    uint8_t reg_value;

    // Leer el registro de configuración A (Sesgo)
    i2c_read_reg(self->i2c.instance, self->i2c.address, HMC5883L_REG_CONFIG_A, &reg_value, 1); // Registro de Configuración A
    reg_value &= 0xFC;  // Limpiar los bits de sesgo (bits [1:0])
    reg_value |= bias;  // Establecer el nuevo valor del sesgo

    // Escribir de vuelta en el registro de Configuración A
    uint8_t data[2] = {HMC5883L_REG_CONFIG_A, reg_value};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void magnetometer_set_range(magnetometer_t *self, enum MAGNETOMETRO_RANGE range)
{
    uint8_t reg_value;

    // Leer el registro de configuración B (Control de ganancia)
    i2c_read_reg(self->i2c.instance, self->i2c.address, HMC5883L_REG_CONFIG_B, &reg_value, 1); // Registro de Configuración B
    reg_value &= 0x1F;  // Limpiar bits de ganancia
    reg_value |= (range << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1)); // Establecer nuevos bits de ganancia

    // Escribir de vuelta en el registro de Configuración B
    uint8_t data[2] = {HMC5883L_REG_CONFIG_B, reg_value};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);
}

void magnetometer_set_mode(magnetometer_t *self, enum MAGNETOMETRO_MODE mode)
{
    uint8_t reg_value;

    // Leer el registro de modo
    i2c_read_reg(self->i2c.instance, self->i2c.address, HMC5883L_REG_MODE, &reg_value, 1); // Registro de Modo
    reg_value &= 0xFC;  // Limpiar bits de modo
    reg_value |= mode; // Establecer nuevos bits de modo

    // Escribir de vuelta en el registro de Modo
    uint8_t data[2] = {HMC5883L_REG_MODE, reg_value};
    i2c_write_blocking(self->i2c.instance, self->i2c.address, data, 2, false);

    self->mode = mode;
}
