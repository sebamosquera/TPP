#ifndef Sensors_driver_h
#define Sensors_driver_h

#include "pico/stdlib.h"
#include "pico/time.h"

#include <math.h>

#include "mpu6050.h"
#include "magnetometer.h"
#include "config.h"

#define DECLINACION_MAGNETICA -10.0f

#define N_MUESTRAS_GYRO 400
#define N_MUESTRAS_MAGNETO 60

#define ACCEL_OFFSET_X (9.81-10.37)
#define ACCEL_OFFSET_Y (9.81-9.47)
#define ACCEL_OFFSET_Z (9.81-8.43)



#ifdef __cplusplus
extern "C"
{
#endif


// Estructura general del driver de sensores
typedef struct sensors_driver {
    mpu6050_t mpu;               // Estructura del MPU6050
    magnetometer_t magnetometer; // Estructura del magnetómetro


    // Todos estos son para debagguear, luego pueden ser valores volatiles, lo unico que importa es el valor filtrado
    float gyro_roll_angle;
    float gyro_pitch_angle;
    float gyro_yaw_angle;

    float accel_roll_angle;
    float accel_pitch_angle;

    float magnet_azimuth_angle;
    // Todos estos son para debagguear, luego pueden ser valores volatiles, lo unico que importa es el valor filtrado


    // Valores ya filtrados (fusionados)
    float roll_angle;            // Ángulo de roll fusionado
    float pitch_angle;           // Ángulo de pitch fusionado
    float yaw_angle;             // Ángulo de yaw fusionado
    // Valores ya filtrados (fusionados)


    float angulo_azimuth;         // yaw_angle
    float angulo_elevacion;      // pitch_angle

    int32_t sense_time_ms;         // Tiempo de muestreo (en segundos)

    float declinacion_magnetica;

    bool mpu_conectado;
    bool magnetometer_conectado;
} sensors_driver_t;


void sensors_driver_init(sensors_driver_t *self);


// CALIBRACION
void sensors_driver_calibrate_sensors(sensors_driver_t *self, bool calibrar_mpu, bool calibrar_magnetometro);

void sensors_driver_calibrate_mpu(sensors_driver_t *self);

void sensors_driver_calibrate_magneto(sensors_driver_t *self);
// CALIBRACION


// LECTURA
void sensors_driver_read_sensors(sensors_driver_t *self);

void sensors_driver_calcular_angulos_accel(sensors_driver_t *self, float_vector_t *accel);

void sensors_driver_calcular_angulos_gyro(sensors_driver_t *self, float_vector_t *gyro, float Ts);

void sensors_driver_calcular_angulos_magneto(sensors_driver_t *self, int16_vector_t *magneto);

void sensors_driver_fusionar_sensores(sensors_driver_t *self);

float sensors_driver_get_azimuth(sensors_driver_t *self);

float sensors_driver_get_elevacion(sensors_driver_t *self);
// LECTURA


// SETTEO OFFSETS
void sensors_driver_set_mpu_offsets(sensors_driver_t *self, float gyro_offsets[3]);

void sensors_driver_get_mpu_gyro_offsets(sensors_driver_t *self, float *offsets);

void sensors_driver_set_magnetometer_offsets(sensors_driver_t *self, int16_t magneto_offsets[3]);

void sensors_driver_get_magnetometer_offsets(sensors_driver_t *self, int16_t *magneto_offsets);
// SETTEO OFFSETS


#ifdef __cplusplus
}
#endif
#endif
