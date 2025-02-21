#include "sensors_driver.h"

void sensors_driver_init(sensors_driver_t *self) {

  self->sense_time_ms = 0;
  self->declinacion_magnetica = DECLINACION_MAGNETICA;

  if( mpu6050_init(&self->mpu, I2C_INSTANCE_MPU))
  {
    self->mpu_conectado = true;
  }
  else
  {
    self->mpu_conectado = false;
  }

  if( magnetometer_init(&self->magnetometer, I2C_INSTANCE_MAGNET))
  {
    self->magnetometer_conectado = true;
  }
  else
  {
    self->magnetometer_conectado = false;
  }
}


// CALIBRACION
void sensors_driver_calibrate_sensors(sensors_driver_t *self, bool calibrar_mpu, bool calibrar_magnetometro) {

  if(self->mpu_conectado && calibrar_mpu)
  {
    sensors_driver_calibrate_mpu(self);
  }

  if(self->magnetometer_conectado && calibrar_magnetometro)
  {
    sensors_driver_calibrate_magneto(self);
  }

}

void sensors_driver_calibrate_mpu(sensors_driver_t *self) {
  self->mpu.calibrated = false;

  float gyro_offset_x = 0.0f;
  float gyro_offset_y = 0.0f;
  float gyro_offset_z = 0.0f;

  // mpu6050_t *mpu6050 = &self->mpu;
  float_vector_t accel;
  float_vector_t gyro;

  for(int i = 0; i < N_MUESTRAS_GYRO; i++) {
    mpu6050_read(&self->mpu, &gyro, &accel);

    gyro_offset_x += gyro.x;
    gyro_offset_y += gyro.y;
    gyro_offset_z += gyro.z;

    sleep_ms(50);
  }

  gyro_offset_x /= N_MUESTRAS_GYRO;
  gyro_offset_y /= N_MUESTRAS_GYRO;
  gyro_offset_z /= N_MUESTRAS_GYRO;

  float gyro_offsets[3] = {gyro_offset_x, gyro_offset_y, gyro_offset_z};
  mpu6050_set_gyro_offsets(&self->mpu, gyro_offsets);

  float accel_offsets[3] = {ACCEL_OFFSET_X, ACCEL_OFFSET_Y, ACCEL_OFFSET_Z};
  mpu6050_set_accel_offsets(&self->mpu, accel_offsets);

  self->mpu.calibrated = true;
}

void sensors_driver_calibrate_magneto(sensors_driver_t *self) {
  int16_t x_min, y_min, z_min;
  int16_t x_max, y_max, z_max;

  x_min = y_min = z_min = 32767;
  x_max = y_max = z_max = -32767;

  magnetometer_t *magnetometer = &self->magnetometer;
  float_vector_t mag;

  // Realizar la calibración  60 * 250ms = 15 segundos
  for (int i = 0; i < N_MUESTRAS_MAGNETO; i++) {

    magnetometer_read(magnetometer, &mag);

    if(mag.x > x_max) {
      x_max = mag.x;
    }

    if(mag.x < x_min) {
      x_min = mag.x;
    }

    if(mag.y > y_max) {
      y_max = mag.y;
    }

    if(mag.y < y_min) {
      y_min = mag.y;
    }

    if(mag.z > z_max) {
      z_max = mag.z;
    }

    if(mag.z < z_min) {
      z_min = mag.z;
    }

    sleep_ms(250);
  }

  int16_t mag_offset_x = (int16_t)(x_max + x_min) / 2;
  int16_t mag_offset_y = (int16_t)(y_max + y_min) / 2;
  int16_t mag_offset_z = (int16_t)(z_max + z_min) / 2;

  int16_t mag_offsets[3] = {mag_offset_x, mag_offset_y, mag_offset_z};

  magnetometer_set_magnet_offsets(magnetometer, mag_offsets);

  self->magnetometer.calibrated = true;
  // algun chequeo de los offsets si estan mal que devuelva false
}
// CALIBRACION


// LECTURA
void sensors_driver_read_sensors(sensors_driver_t *self) {

  float Ts = (float)(us_to_ms(time_us_64()) - self->sense_time_ms) / 1000.0;
  self->sense_time_ms = us_to_ms(time_us_64());

  if(self->mpu_conectado) {

    mpu6050_t *mpu6050 = &self->mpu;
    float_vector_t accel;
    float_vector_t gyro;
    mpu6050_read(mpu6050, &gyro, &accel);

    sensors_driver_calcular_angulos_gyro(self, &gyro, Ts);
    sensors_driver_calcular_angulos_accel(self, &accel);

  }

  if(self->magnetometer_conectado) {

    magnetometer_t *magnetometer = &self->magnetometer;
    float_vector_t mag;
    magnetometer_read(magnetometer, &mag);

    sensors_driver_calcular_angulos_magneto(self, &mag);

  }

  sensors_driver_fusionar_sensores(self);

}

void sensors_driver_calcular_angulos_accel(sensors_driver_t *self, float_vector_t *accel) {
  float accel_x = accel->x;
  float accel_y = accel->y;
  float accel_z = accel->z;

  self->accel_roll_angle = atan2f(accel_y, sqrt( pow(accel_x, 2) + pow(accel_z, 2) ) ) * 180 / M_PI;
  self->accel_pitch_angle = - atan2f(-accel_x, sqrt( pow(accel_y, 2) + pow(accel_z, 2) ) ) * 180 / M_PI;
}

void sensors_driver_calcular_angulos_gyro(sensors_driver_t *self, float_vector_t *gyro, float Ts) {
  self->gyro_roll_angle  = self->roll_angle  + gyro->x * Ts;
  self->gyro_pitch_angle = self->pitch_angle - gyro->y * Ts;
  self->gyro_yaw_angle   = self->yaw_angle   - gyro->z * Ts;

  float angulo_normalizado = fmod(self->gyro_yaw_angle, 360.0); // Resto de la división por 360
  if (angulo_normalizado < 0) {
    angulo_normalizado += 360.0; // Ajusta para que esté entre 0 y 359.99
  }
  self->gyro_yaw_angle = angulo_normalizado;
}

void sensors_driver_calcular_angulos_magneto(sensors_driver_t *self, float_vector_t *magneto) {

  float azimuth = atan2f( - magneto->x*1000, - magneto->y*1000) * 180 / M_PI;
  azimuth += self->declinacion_magnetica;

  azimuth = fmod(azimuth, 360.0); // Resto de la división por 360

  if (azimuth < 0) {
    azimuth += 360; // Ajustar a rango [0, 360]
  }

  if (azimuth >= 0 && azimuth <= 360) {
    self->magnet_azimuth_angle = azimuth;
  }
}

void sensors_driver_fusionar_sensores(sensors_driver_t *self) {

  // FILTRADO
  float alpha = 0.9;

  if(self->mpu_conectado)
  {
    // FILTRADO
    self->roll_angle = alpha * self->gyro_roll_angle + (1 - alpha) * self->accel_roll_angle;
    self->pitch_angle = alpha * self->gyro_pitch_angle + (1 - alpha) * self->accel_pitch_angle;

    self->yaw_angle = (1 - alpha) * self->yaw_angle + alpha * self->gyro_yaw_angle; // le doy un valor, en caso que el magnetometro no este conectado
  }

  if(self->magnetometer_conectado && self->mpu_conectado)
  {
    // condicion por discontinuidad 360 = 0. ejemplo: gyro = 359 y magneto = 1 (magneto = 361)
    float alpha2 = 0.9;
    float yaw_X_comp_filtrado = alpha2 * cos(self->gyro_yaw_angle * M_PI / 180.0) + (1 - alpha2) * cos(self->magnet_azimuth_angle * M_PI / 180.0);
    float yaw_Y_comp_filtrado = alpha2 * sin(self->gyro_yaw_angle * M_PI / 180.0) + (1 - alpha2) * sin(self->magnet_azimuth_angle * M_PI / 180.0);
    float yaw_angle_filtrado = atan2(yaw_Y_comp_filtrado, yaw_X_comp_filtrado) * 180.0 / M_PI;
    if (yaw_angle_filtrado < 0) {
      yaw_angle_filtrado += 360.0;
    }

    self->yaw_angle = yaw_angle_filtrado;
  }

  if(self->magnetometer_conectado && !self->mpu_conectado)
  {
    self->roll_angle = 0;
    self->pitch_angle = 0;
    self->yaw_angle = alpha * self->yaw_angle + (1 - alpha) * self->magnet_azimuth_angle;
  }
  // FILTRADO

  // ACTUALIZACION DE ANGULOS DEL TRACKER
  self->angulo_azimuth = self->yaw_angle;
  self->angulo_elevacion = self->pitch_angle;
}


float sensors_driver_get_azimuth(sensors_driver_t *self) {
  return self->angulo_azimuth;
}

float sensors_driver_get_elevacion(sensors_driver_t *self) {
  return self->angulo_elevacion;
}
// LECTURA


// SETTEO OFFSETS
void sensors_driver_set_mpu_offsets(sensors_driver_t *self, float gyro_offsets[3]) {
  mpu6050_set_gyro_offsets(&self->mpu, gyro_offsets);

  float accel_offsets[3] = {ACCEL_OFFSET_X, ACCEL_OFFSET_Y, ACCEL_OFFSET_Z};
  mpu6050_set_accel_offsets(&self->mpu, accel_offsets);

  self->mpu.calibrated = true;
}

void sensors_driver_get_mpu_gyro_offsets(sensors_driver_t *self, float offsets[3]) {
  offsets[0] = self->mpu.gyro_offsets.x;
  offsets[1] = self->mpu.gyro_offsets.y;
  offsets[2] = self->mpu.gyro_offsets.z;
}

void sensors_driver_set_magnetometer_offsets(sensors_driver_t *self, int16_t mag_offsets[3]) {
  magnetometer_set_magnet_offsets(&self->magnetometer, mag_offsets);

  self->magnetometer.calibrated = true;
}

void sensors_driver_get_magnetometer_offsets(sensors_driver_t *self, int16_t *magneto_offsets) {
  magnetometer_get_magnet_offsets(&self->magnetometer, magneto_offsets);

  self->mpu.calibrated = true;
}
// SETTEO OFFSETS
