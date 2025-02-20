#include "antena_tracker.h"

void antena_tracker_init(antena_tracker_t *self, sensors_driver_t *sensors_driver, motor_t *motor_azimuth, motor_t *motor_elevacion) {
  self->sensors_driver = *sensors_driver;

  self->motor_azimuth = *motor_azimuth;
  self->motor_elevacion = *motor_elevacion;

  self->ratio_azimuth = RATIO_AZIMUTH;
  self->ratio_elevacion = RATIO_ELEVACION;

  self->latitud_tracker = -34.5964340; // 9 digitos
  self->longitud_tracker = -58.4883492;
  self->altura_tracker = 0;

  self->latitud_drone = -34.5964340;
  self->longitud_drone = -58.4883492;
  self->altura_drone = 0;


  self->angulo_azimuth_referencia = 0; // r1
  self->angulo_elevacion_referencia = 0; // r2

  self->angulo_azimuth_real = 0; // y1
  self->angulo_elevacion_real = 0; // y2

  // self->accion_control_azimuth.angulo_giro = 0; // u1
  // self->accion_control_azimuth.sentido_giro = 0; // u1
  //
  // self->accion_control_elevacion.angulo_giro = 0; // u2
  // self->accion_control_elevacion.sentido_giro = 0; // u2

  self->angulo_giro_azimuth = 0;
  self->angulo_giro_elevacion = 0;

}


/*
// Recibe r1, r2, y1, y2
// Devuelve u1, u2
void _calcular_accion_control(float ang_az_ref, float ang_el_ref, float ang_az_real, float ang_el_real, float *angulo_giro1, bool *sentido_giro1, float *angulo_giro2, bool *sentido_giro2) {

  // accion de control para angulo azimuth
  // EJEMPLO: antena apuntando al SUR: angulo_azimuth = 180
  //          drone justo al OESTE:    angulo_azimuth_referencia = 270
  //                                   angulo_giro = +90
  *angulo_giro1 = fmod((ang_az_ref - ang_az_real + 360), 360); // Asegura valores positivos

  if (*angulo_giro1 <= 180) {
    *sentido_giro1 = 0; // Horario
  } else {
    *angulo_giro1 = 360 - *angulo_giro1;
    *sentido_giro1 = 1; // Antihorario
  }


  // accion de control para angulo de elevacion
  if (ang_el_real <= ang_el_ref) {
      *angulo_giro2 = ang_el_ref - ang_el_real; // Sentido Horario
      *sentido_giro2 = 1;
  } else {
      *angulo_giro2 = ang_el_real - ang_el_ref; // *sentido_giro Antihorario
      *sentido_giro2 = 0;
  }
}

*/



void antena_tracker_calcular_accion_control(antena_tracker_t *self) {

  float ang_az_ref = self->angulo_azimuth_referencia;
  float ang_az_real = self->angulo_azimuth_real;

  self->angulo_giro_azimuth = fmod((ang_az_ref - ang_az_real + 360), 360); // Asegura valores positivos


  float ang_el_ref = self->angulo_elevacion_referencia;
  float ang_el_real = self->angulo_elevacion_real;

  self->angulo_giro_elevacion = ang_el_ref - ang_el_real; // Asegura valores positivos
}


void antena_tracker_ejecutar_accion_control(antena_tracker_t *self) {

  float angulo_giro1 = 0;
  int sentido_giro1 = 0;
  if(self->angulo_giro_azimuth <= 180)
  {
    sentido_giro1 = 0;
    angulo_giro1 = self->angulo_giro_azimuth;
  }
  else
  {
    sentido_giro1 = 1;
    angulo_giro1 = 360 - self->angulo_giro_azimuth;
  }

  motor_mover(&self->motor_azimuth, angulo_giro1 * self->ratio_azimuth, sentido_giro1);


  float angulo_giro2 = 0;
  int sentido_giro2 = 0;
  if(self->angulo_giro_elevacion >= 0)
  {
    sentido_giro2 = 1;
    angulo_giro2 = self->angulo_giro_elevacion;
  }
  else
  {
    sentido_giro2 = 0;
    angulo_giro2 = -self->angulo_giro_elevacion;
  }

  // float velocidad = angulo_giro2 * 2.5;
  // motor_set_velocidad(&self->motor_elevacion, velocidad);

  motor_mover(&self->motor_elevacion, angulo_giro2 * self->ratio_elevacion, sentido_giro2);

}



// ---------------------------   REFERENCIA
void antena_tracker_actualizar_referencia(antena_tracker_t *self) {
  // El angulo azimuth de referencia se calcula en base a las latitudes y longitudes. No tiene en cuenta a donde esta apuntando el tracker

  // Con ultima posicion obtenida del drone y la posicion del Tacker, calcular angulos de referencia: azimut y elevacion
  float angulo_azimut_referencia_ = antena_tracker_calcular_angulo_azimuth_referencia(self->latitud_tracker, self->longitud_tracker, self->latitud_drone, self->longitud_drone);

  // quizas algun chequeo sobre el angulo_azimut_referencia
  self->angulo_azimuth_referencia = angulo_azimut_referencia_;


  float angulo_elevacion_referencia_ = antena_tracker_calcular_angulo_elevacion_referencia(self, self->altura_tracker, self->altura_drone);

  // quizas algun chequeo
  self->angulo_elevacion_referencia = angulo_elevacion_referencia_;
}

float antena_tracker_calcular_angulo_azimuth_referencia(double lat1, double long1, double lat2, double long2) {

  // Convertir a radianes
  lat1 = lat1 * (M_PI / 180.0);
  long1 = long1 * (M_PI / 180.0);
  lat2 = lat2 * (M_PI / 180.0);
  long2 = long2 * (M_PI / 180.0);

  // Diferencia de longitudes
  double dlong = long2 - long1;

  // Cálculo del azimut
  double x = sin(dlong) * cos(lat2);
  double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlong);
  float angulo_azimut_ = atan2(x, y) * (180.0 / M_PI);

  // Ajustar a positivo entre 0° y 360°
  if (angulo_azimut_ < 0) {
    angulo_azimut_ += 360;
  }

  return angulo_azimut_;
}

// chequear
float antena_tracker_calcular_angulo_elevacion_referencia(antena_tracker_t *self, double altura1, double altura2) {

  double distancia_haversine = antena_tracker_calcular_distancia_haversine(self->latitud_tracker, self->longitud_tracker, self->latitud_drone, self->longitud_drone);
  double altura_ajustada = antena_tracker_ajustar_altura_curvatura(distancia_haversine, self->altura_drone);

  // float angulo_elevacion_referencia = calcular_angulo_elevacion(this->altura, this->altura_drone, distancia_haversine);
  float angulo_elevacion_referencia = antena_tracker_calcular_angulo_elevacion(self->altura_tracker, altura_ajustada, distancia_haversine);

  return angulo_elevacion_referencia;
}

// Función para calcular la distancia Haversine en metros
double antena_tracker_calcular_distancia_haversine(double lat1, double lon1, double lat2, double lon2) {
    // Radio de la Tierra en metros
    double R = 6371.0 * 1000;

    // Convertir grados a radianes
    lat1 = lat1 * (M_PI / 180.0);
    lon1 = lon1 * (M_PI / 180.0);
    lat2 = lat2 * (M_PI / 180.0);
    lon2 = lon2 * (M_PI / 180.0);

    // Diferencia de latitudes y longitudes
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Fórmula de Haversine
    double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double distancia = R * c;
    return distancia;
}

// Función para ajustar la altura considerando la curvatura de la Tierra
double antena_tracker_ajustar_altura_curvatura(double dist_haversine, double alt_drone) {
    double R = 6371 * 1000;

    // Calcular la caída de la curvatura de la Tierra
    double caida_curvatura = (dist_haversine * dist_haversine) / (2 * R);

    double altura_ajustada = alt_drone - caida_curvatura;

    return altura_ajustada;
}

// Función para calcular el ángulo de elevación
float antena_tracker_calcular_angulo_elevacion(double altura_tracker, double altura_drone, double distancia_haversine) {
    double diferencia_altura = altura_drone - altura_tracker;

    // Calcular el ángulo de elevación (en grados)
    float angulo_elevacion = atan2(diferencia_altura, distancia_haversine) * (180.0 / M_PI);

    return angulo_elevacion;
}
// ---------------------------   REFERENCIA


// SETTERS
void antena_tracker_set_posicion_del_drone(antena_tracker_t *self, double lat_drone, double lon_drone, double altura_drone) {
  self->latitud_drone = lat_drone;
  self->longitud_drone = lon_drone;
  self->altura_drone = altura_drone;
}

void antena_tracker_set_altura_del_drone(antena_tracker_t *self, double altura_drone) {
  self->altura_drone = altura_drone;
}

void antena_tracker_set_posicion_del_tracker(antena_tracker_t *self, double lat_tracker, double lon_tracker, double altura_tracker) {
  self->latitud_tracker = lat_tracker;
  self->longitud_tracker = lon_tracker;
  self->altura_tracker = altura_tracker;
}
// SETTERS


// TIMER DE SENSADO
void antena_tracker_iniciar_timer_sensores(antena_tracker_t *self, int32_t frecuencia_timer) {
  if (!add_repeating_timer_ms(frecuencia_timer, repeating_timer1_callback, self, &self->sensorTimer)) {
    // Serial.println("Error al configurar el timer");
  }
  else {
    // Serial.println("Timer de sensores configurado correctamente");
  }
}

bool repeating_timer1_callback(__unused struct repeating_timer *t) {
  if(t->user_data) {
    antena_tracker_t *self = (antena_tracker_t *)(t->user_data);

    actualizar_angulos(self);
  }

  return true; // Repite el temporizador
}

// CALLBACK 1
// callback de timer1 llama a esta funcion
void actualizar_angulos(antena_tracker_t *self) {
  sensors_driver_read_sensors(&self->sensors_driver);
  float angulo_azimuth = sensors_driver_get_azimuth(&self->sensors_driver);
  float angulo_elevacion = sensors_driver_get_elevacion(&self->sensors_driver);

  self->angulo_azimuth_real = angulo_azimuth;
  self->angulo_elevacion_real = angulo_elevacion;
  //
  // Serial.print("Angulo Azimuth: ");
  // Serial.println(angulo_azimuth);
  // Serial.print("Angulo Elevacion: ");
  // Serial.println(angulo_elevacion);
  // Serial.println("------------------");
}


// TIMER DE CONTROL
void antena_tracker_iniciar_timer_control(antena_tracker_t *self, int32_t frecuencia_timer) {
  // timer2 para calcular accion de control y ejecutar accion de control
  if (!add_repeating_timer_ms(frecuencia_timer, repeating_timer2_callback, self, &self->controlTimer)) {
    // Serial.println("Error al configurar el timer2");
  }
  else {
    // Serial.println("Timer2 configurado correctamente");
  }

  // Inicia el timer "controlTimer" que calcula y ejecuta accion de control cada 1s (1000ms)
}

bool repeating_timer2_callback(__unused struct repeating_timer *t) {
  if(t->user_data) {
    antena_tracker_t *self = (antena_tracker_t *)(t->user_data);

    actualizar_control(self);
  }

  return true; // Repite el temporizador
}

// CALLBACK 2
// callback de timer2 llama a esta funcion
void actualizar_control(antena_tracker_t *self) {
  // antena_tracker_actualizar_referencia(self);
  antena_tracker_calcular_accion_control(self);
  antena_tracker_ejecutar_accion_control(self);

  // Serial.println("");
  // Serial.println("---------    CONTROL   ---------");
  // Serial.print("     Angulo giro azimuth motor:   ");
  // Serial.println(self->accion_control_azimuth.angulo_giro);
  // Serial.print("      Angulo giro Elevacion motor:   ");
  // Serial.println(self->accion_control_elevacion.angulo_giro);
  // Serial.println("---------    ***    ---------");
  // Serial.println("");

}
