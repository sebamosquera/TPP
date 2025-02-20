#include "motor.h"

void motor_init(motor_t *self, float velocidad, uint pin_direccion, uint pin_step, int pasos_por_vuelta) {

  self->velocidad = velocidad;
  self->pin_direccion = pin_direccion;
  self->pin_step = pin_step;
  self->pasos_por_vuelta = pasos_por_vuelta;

  self->en_movimiento = false;

  // Inicializar el pin
  gpio_init(pin_direccion);

  // Configurarlo como salida
  gpio_set_dir(pin_direccion, GPIO_OUT);

  // Configurar la funcion del pin: PWM
  gpio_set_function(pin_step, GPIO_FUNC_PWM);

  motor_set_velocidad(self, velocidad);
}


void motor_set_velocidad(motor_t *self, float velocidad) {
  uint slice_num = pwm_gpio_to_slice_num(self->pin_step);

  // El DIV es siempre el mismo. EL valor maximo de DIV
  float div = 255.99999;
  pwm_set_clkdiv(slice_num, div);

  uint32_t clock_frequency = 133000000;
  // Utilizar la constante: CLOCK_FREQUENCY


  // El set_wrap configura el periodo de la senal PWM
  if(velocidad < VELOCIDAD_MINIMA_MOTOR)
  {
    velocidad = VELOCIDAD_MINIMA_MOTOR;
  }
  else if(velocidad > VELOCIDAD_MAXIMA_MOTOR)
  {
    velocidad = VELOCIDAD_MAXIMA_MOTOR;
  }

  // El set_wrap configura el periodo de la senal PWM
  uint32_t top = (clock_frequency / (div * velocidad));
  pwm_set_wrap(slice_num, top);

  // duty cycle al 50%
  uint32_t cc = top * 0.5;
  pwm_set_gpio_level(self->pin_step, cc);


  self->velocidad = velocidad;
}

// Recibe un angulo, una direccion. Calcula los pasos, llama a la funcion iniciar_movimiento
void motor_mover(motor_t *self, float angulo, int direccion) {

  gpio_put(self->pin_direccion, direccion);

  uint32_t pasos = (uint32_t) round((angulo - 0.0) * (self->pasos_por_vuelta - 0.0) / (360.0 - 0.0) + 0.0);

  uint32_t frecuencia_pwm = self->velocidad;

  motor_iniciar_movimiento(self, pasos, frecuencia_pwm);
}

// Activa el pwm y la alarma. Recibe cantidad de pasos y frecuencia_pwm (velocidad del motor) calcula tiempo de alarma.
void motor_iniciar_movimiento(motor_t *self, uint32_t pasos, uint32_t frecuencia_pwm) {

  uint slice_num = pwm_gpio_to_slice_num(self->pin_step);

  if(self->en_movimiento) {
    motor_finalizar_movimiento(self);
  }

  // tiempo de alarma = duracion de movimiento del motor
  uint32_t tiempo_alarma = (uint32_t) (pasos * 1000 / frecuencia_pwm);

  if(tiempo_alarma>0) {
    add_alarm_in_ms(tiempo_alarma, motor_alarm_callback, self, false);
    pwm_set_enabled(slice_num, true);
    self->en_movimiento = true;
  }
  else {
    // Serial.println("MOTOR_INICIAR_MOVIMIENTO: Alarma no iniciada");
  }
}

// Apaga la alarma. Llama a la funcion motor_finalizar_movimiento
int64_t motor_alarm_callback(alarm_id_t id, __unused void *user_data) {
  if (user_data) {
      motor_t *self = (motor_t *)user_data; // Convertir el puntero genérico a motor_t
      motor_finalizar_movimiento(self);    // Llamar a la función de finalizar movimiento
  }
  return 0; // Retorna 0 para no reprogramar la alarma
}

// Desactiva el pwm
void motor_finalizar_movimiento(motor_t *self) {

  uint slice_num = pwm_gpio_to_slice_num(self->pin_step);

  pwm_set_enabled(slice_num, false);

  self->en_movimiento = false;
}
