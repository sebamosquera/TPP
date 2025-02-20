#ifndef Motor_h
#define Motor_h

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "math.h"

#include <stdio.h>

#include <Arduino.h>


#ifdef __cplusplus
extern "C"
{
#endif


#define CLOCK_FREQUENCY 133000000

#define VELOCIDAD_MAXIMA_MOTOR 200  // 200Hz ()
// #define VELOCIDAD_MINIMA_MOTOR 16   // 16Hz ()
#define VELOCIDAD_MINIMA_MOTOR 25 


typedef struct motor
{
  float velocidad;
  uint pin_direccion;
  uint pin_step;
  int pasos_por_vuelta;
  bool en_movimiento;
} motor_t;


void motor_init(motor_t *self, float velocidad, uint pin_direccion, uint pin_step, int steps_por_vuelta);

void motor_set_velocidad(motor_t *self, float velocidad);

void motor_mover(motor_t *self, float angulo, int sentido);

void motor_iniciar_movimiento(motor_t *self, uint32_t pasos, uint32_t frecuencia_pwm);

void motor_finalizar_movimiento(motor_t *self);

int64_t motor_alarm_callback(alarm_id_t id, __unused void *user_data);


#ifdef __cplusplus
}
#endif
#endif
