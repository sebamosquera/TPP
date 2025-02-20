#ifndef Antena_tracker_h
#define Antena_tracker_h

#include <Arduino.h>

#include "motor.h"
#include "sensors_driver.h"
#include "config.h"


#ifdef __cplusplus
extern "C"
{
#endif


typedef struct antena_tracker
{
    sensors_driver_t sensors_driver;

    motor_t motor_azimuth;
    motor_t motor_elevacion;

    float ratio_azimuth;
    float ratio_elevacion;


    double latitud_tracker;
    double longitud_tracker;
    double altura_tracker;

    double latitud_drone;
    double longitud_drone;
    double altura_drone;


    float angulo_azimuth_referencia; // r1
    float angulo_elevacion_referencia; // r2

    float angulo_azimuth_real; // y1
    float angulo_elevacion_real; // y2

    float angulo_giro_azimuth; // u1
    float angulo_giro_elevacion; // u2

    
    repeating_timer_t sensorTimer; // Timer1 periódico. Lee sensores, actualiza angulos azimut y elevacion
    repeating_timer_t controlTimer; // Timer2 periódico. Calcula accion de control (angulo y sentido de giro para motores), ejecuta accion de control (mover motores)

} antena_tracker_t;


void antena_tracker_init(antena_tracker_t *self, sensors_driver_t *sensors_driver, motor_t *motor_azimuth, motor_t *motor_elevacion);

// ACCION DE CONTROL
void antena_tracker_calcular_accion_control(antena_tracker_t *self);

void antena_tracker_ejecutar_accion_control(antena_tracker_t *self);
// ACCION DE CONTROL


// REFERENCIA
void antena_tracker_actualizar_referencia(antena_tracker_t *self);

float antena_tracker_calcular_angulo_azimuth_referencia(double lat1, double long1, double lat2, double long2);

float antena_tracker_calcular_angulo_elevacion_referencia(antena_tracker_t *self, double altura1, double altura2);
// REFERENCIA


// AUXILIARES REFERENCIA
double antena_tracker_calcular_distancia_haversine(double lat1, double lon1, double lat2, double lon2);

double antena_tracker_ajustar_altura_curvatura(double dist_haversine, double alt_drone);

float antena_tracker_calcular_angulo_elevacion(double altura_tracker, double altura_drone, double distancia_haversine);
// AUXILIARES REFERENCIA


// SETTERS
void antena_tracker_set_posicion_del_drone(antena_tracker_t *self, double lat_drone, double lon_drone, double altura_drone); // al oeste y norte

void antena_tracker_set_posicion_del_tracker(antena_tracker_t *self, double lat_tracker, double lon_tracker, double altura_tracker);

void antena_tracker_set_altura_del_drone(antena_tracker_t *self, double altura_tracker);
// SETTERS


// TIMERS

// TIMER DE SENSADO
void antena_tracker_iniciar_timer_sensores(antena_tracker_t *self, int32_t frecuencia_timer);

bool repeating_timer1_callback(__unused struct repeating_timer *t); // callback de interrupcion

void actualizar_angulos(antena_tracker_t *self); // callback de timer2 llama a esta funcion

// TIMER DE CONTROL
void antena_tracker_iniciar_timer_control(antena_tracker_t *self, int32_t frecuencia_timer);

bool repeating_timer2_callback(__unused struct repeating_timer *t); // callback de interrupcion

void actualizar_control(antena_tracker_t *self); // callback de timer2 llama a esta funcion





#ifdef __cplusplus
}
#endif
#endif
