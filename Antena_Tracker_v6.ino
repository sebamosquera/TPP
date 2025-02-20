#include "./c_library_v2-master/common/mavlink.h"

#include "antena_tracker.h"
#include "motor.h"
#include "sensors_driver.h"


// --- PARA MEJORAR
// Definir rangos y sentidos: Por ejemplo el rango del angulo del acelerometro esta entre -87 y +87 y eso limita bastante
// Mejorar tema prints
// ---

sensors_driver_t sensors_driver;

antena_tracker_t antena_tracker;

motor_t motor_azimuth;
motor_t motor_elevacion;

uint32_t frecuencia_timer_sensores = 10;  // 10  ms   <=>  100Hz
uint32_t frecuencia_timer_control = 1000;  // 100 ms (en realidad no es la frecuencia, es cada cuanto se ejecuta) (1Hz => 1000ms)


bool calibrar_mpu = false;
bool calibrar_magnetometro = false;


// Variables de temporización
absolute_time_t tiempo_anterior_10ms = get_absolute_time();
absolute_time_t tiempo_anterior_100ms = get_absolute_time();
const int intervalo_10ms = 10;   // Intervalo de 10 ms
const int intervalo_100ms = 100; // Intervalo de 100 ms



float elevacion_ref = 0;
int sentido_giro = 0;


void on_uart_rx();


void setup() {
  Serial.begin(57600);

  iniciar_i2c0();
  iniciar_i2c1();

  iniciar_uart0();

  sleep_ms(5000);

  // Serial.println("Inicia Raspberry");

  motor_init(&motor_azimuth, 50, PIN_DIR_1, PIN_PWM_1, PASOS_POR_VUELTA);
  motor_init(&motor_elevacion, 50, PIN_DIR_2, PIN_PWM_2, PASOS_POR_VUELTA);

  // Inicializar el driver y  los sensores
  sensors_driver_init(&sensors_driver);

  sensors_driver_calibrate_sensors(&sensors_driver, calibrar_mpu, calibrar_magnetometro);

  if( !calibrar_mpu )
  {
    float gyro_ffsets[3] = {-2.49f, -0.43f, -0.88f};
    sensors_driver_set_mpu_offsets(&sensors_driver, gyro_ffsets);
  }

  if(sensors_driver.mpu.calibrated)
  {
    float offsets[3] = {0};
    sensors_driver_get_mpu_gyro_offsets(&sensors_driver, offsets);

    // Serial.print("Offset gyro x: \t");
    // Serial.println(offsets[0]);
    //
    // Serial.print("Offset gyro y: \t");
    // Serial.println(offsets[1]);
    //
    // Serial.print("Offset gyro z: \t");
    // Serial.println(offsets[2]);
    // Serial.println();
  }


  if( !calibrar_magnetometro )
  {
    int16_t magneto_offsets[3] = {8, 244, -138};
    sensors_driver_set_magnetometer_offsets(&sensors_driver, magneto_offsets);
  }

  if(sensors_driver.magnetometer.calibrated)
  {
    int16_t magneto_offsets[3] = {0};
    sensors_driver_get_magnetometer_offsets(&sensors_driver, magneto_offsets);

    // Serial.print("Offset magnet x: \t");
    // Serial.println(magneto_offsets[0]);
    //
    // Serial.print("Offset magnet y: \t");
    // Serial.println(magneto_offsets[1]);
    //
    // Serial.print("Offset magnet z: \t");
    // Serial.println(magneto_offsets[2]);
    // Serial.println();
  }

  sleep_ms(1000);


  // sensors_driver_set_initial_gyro_angles(&sensors_driver); // NO VA
  //


  antena_tracker_init(&antena_tracker, &sensors_driver, &motor_azimuth, &motor_elevacion);


  //
  // antena_tracker_set_posicion_del_drone(&antena_tracker, -34.5964800, -58.4881500, 1);
  // antena_tracker_set_posicion_del_tracker(&antena_tracker, -34.5964800,  -58.488200, 0);

  antena_tracker_set_posicion_del_drone(&antena_tracker, -34.5964800, -58.488184, 0); // Quedan separados unos metros
  antena_tracker_set_posicion_del_tracker(&antena_tracker, -34.5964800,  -58.488200, 0);
  //


  // Iniciar los timers de sensores cada 100Hz y de control cada 10Hz
  antena_tracker_iniciar_timer_sensores(&antena_tracker, frecuencia_timer_sensores);

  sleep_ms(5000);
  // Serial.println("Inicia el timer de control");

  antena_tracker_iniciar_timer_control(&antena_tracker, frecuencia_timer_control);
  //

  // Habilitar interrupciones UART
  // uart_set_irq_enables(uart0, true, false); // Habilitar interrupción de recepción
  // irq_set_exclusive_handler(UART0_IRQ, on_uart_rx); // Establecer el manejador`
  // irq_set_enabled(UART0_IRQ, true); // Habilitar la interrupción
  //

  // sleep_ms(2000);
  // Serial.println("Mover motor");
  // motor_mover(&motor_azimuth, 30*4.55,0);
  // sleep_ms(5000);
  // Serial.println("Mover motor 2");
  // motor_mover(&motor_azimuth, 30*4.55,1);


  // sleep_ms(5000);
  // Serial.println("Mover motor Elevacion 30 grados");
  // motor_mover(&motor_elevacion, 30*4,0);
  // sleep_ms(5000);
  // Serial.println("Mover motor Elevacion -30 grados");
  // motor_mover(&motor_elevacion, 30*4,1);

   // antena_tracker.angulo_elevacion_referencia = 80;

}



void loop() {

  absolute_time_t tiempo_inicial = get_absolute_time();

  if(Serial.available()>0) {
    elevacion_ref = (float)Serial.read() - 70;
  }

  // antena_tracker.angulo_azimuth_referencia = elevacion_ref;
  antena_tracker.angulo_elevacion_referencia = elevacion_ref;

  // motor_mover(&motor_elevacion, elevacion_ref, sentido_giro);

  float azim_real = antena_tracker.angulo_azimuth_real;
  float elev_real = antena_tracker.angulo_elevacion_real;
  matlab_send(azim_real,elev_real,elevacion_ref + 70);

  absolute_time_t tiempo_final = get_absolute_time();

  int64_t diferencia = absolute_time_diff_us(tiempo_final, tiempo_inicial) / 1000;
  sleep_ms(20 - diferencia);




  // sleep_ms(5000);
  // Serial.println("Mover motor Elevacion -15 grados");
  // motor_mover(&motor_elevacion, 15*4,0);
  // sleep_ms(5000);
  // Serial.println("Mover motor Elevacion -10 grados");
  // motor_mover(&motor_elevacion, 10*4,0);

  // sleep_ms(10000);
  // Serial.println("Mover motor Elevacion 45 grados");
  // motor_mover(&motor_elevacion, 45*4,1);




  // absolute_time_t tiempo_actual = get_absolute_time();
  //
  // // Diferencia para el intervalo de 10 ms
  // int64_t diferencia_10ms = absolute_time_diff_us(tiempo_anterior_10ms, tiempo_actual) / 1000;
  // if (diferencia_10ms >= intervalo_10ms) {
  //     sensors_driver_read_sensors(&sensors_driver);
  //     tiempo_anterior_10ms = tiempo_actual;
  // }
  //
  // // Diferencia para el intervalo de 100 ms
  // int64_t diferencia_100ms = absolute_time_diff_us(tiempo_anterior_100ms, tiempo_actual) / 1000;
  // if (diferencia_100ms >= intervalo_100ms) {
  //
  //     // Serial.print("Accel Roll: ");
  //     // Serial.print(sensors_driver.accel_roll_angle);
  //     // Serial.print("\tAccel Pitch: ");
  //     // Serial.println(sensors_driver.accel_pitch_angle);
  //     //
  //     // Serial.print("Gyro Roll: ");
  //     // Serial.print(sensors_driver.gyro_roll_angle);
  //     // Serial.print("\tGyro Pitch: ");
  //     // Serial.print(sensors_driver.gyro_pitch_angle);
  //     // Serial.print("\tGyro Yaw: ");
  //     // Serial.println(sensors_driver.gyro_yaw_angle);
  //     //
  //     Serial.print("Roll: ");
  //     Serial.print(sensors_driver.roll_angle);
  //     Serial.print("\tPitch: ");
  //     Serial.print(sensors_driver.pitch_angle);
  //     Serial.print("\tYaw: ");
  //     Serial.println(sensors_driver.yaw_angle);
  //     Serial.println("----------------------");
  //
  //     // Serial.print("Magnet Azimuth: ");
  //     // Serial.println(sensors_driver.magnet_azimuth_angle);
  //     // Serial.println("----------------------");
  //
  //     tiempo_anterior_100ms = tiempo_actual;
  // }


}


void matlab_send(float dato1, float dato2, float dato3){
  Serial.write("abcd");
  byte * b = (byte *) &dato1;
  Serial.write(b,4);
  b = (byte *) &dato2;
  Serial.write(b,4);
  b = (byte *) &dato3;
  Serial.write(b,4);
}



void iniciar_i2c0() {

  gpio_init(PIN_MPU_SDA);
  gpio_init(PIN_MPU_SCL);
  gpio_set_function(PIN_MPU_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_MPU_SCL, GPIO_FUNC_I2C);

  // Don't forget the pull ups! | Or use external ones
  gpio_pull_up(PIN_MPU_SDA);
  gpio_pull_up(PIN_MPU_SCL);

  i2c_init(I2C_INSTANCE_MPU, 100000);
}

void iniciar_i2c1() {

  gpio_init(PIN_MAGNET_SDA);
  gpio_init(PIN_MAGNET_SCL);
  gpio_set_function(PIN_MAGNET_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_MAGNET_SCL, GPIO_FUNC_I2C);

  // Don't forget the pull ups! | Or use external ones
  gpio_pull_up(PIN_MAGNET_SDA);
  gpio_pull_up(PIN_MAGNET_SCL);

  i2c_init(I2C_INSTANCE_MAGNET, 100000);
}


void iniciar_uart0() {
  // configuracion de UART - SERIAL1
  uart_init(UART_INSTANCE_, 57600); // uart0 a 57600 baudios
  gpio_set_function(PIN_TX, GPIO_FUNC_UART); // Pin TX: 0
  gpio_set_function(PIN_RX, GPIO_FUNC_UART); // Pin RX: 1
}

// Actualizar posicion del drone
void on_uart_rx() {

  mavlink_message_t msg;
  mavlink_status_t status;

  // leer mensaje
  while (uart_is_readable(uart0)) {
    char c = uart_getc(uart0); // Leer un carácter del UART

    // Decodifica el mensaje MAVLink
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)){
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
          printf("Decodificando Global Position...");

          mavlink_global_position_int_t global_position_int;
          mavlink_msg_global_position_int_decode(&msg, &global_position_int);

          double latitud_drone = (double) (global_position_int.lat / 1E7);
          double longitud_drone = (double) (global_position_int.lon / 1E7);
          double altitud_drone = (double) (global_position_int.alt / 1E3); // para pasar de milimetros a metros

          // Actualizar posicion del drone
          // antena_tracker_set_posicion_del_drone(&antena_tracker, latitud_drone, longitud_drone, altitud_drone);
          // antena_tracker_set_altura_del_drone(&antena_tracker, altitud_drone);

          // Serial.println("Lee mensaje mavlink Global position:  ");
          // Serial.print("Lat Drone:  ");
          // Serial.println(latitud_drone, 7);
          // Serial.print("Long Drone:  ");
          // Serial.println(longitud_drone, 7);
          // Serial.print("Alt Drone:  ");
          // Serial.println(altitud_drone, 7);
          // Serial.println();


          break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT: {
          //Serial.println("Decodificando Hearbeat...");
          // mavlink_msg_heartbeat_decode();
          break;
        }

        case MAVLINK_MSG_ID_ALTITUDE: {

          mavlink_altitude_t altitude;
          mavlink_msg_altitude_decode(&msg, &altitude);

          float altitude_amsl = altitude.altitude_amsl;       // Altitud sobre el nivel del mar (en metros)
          float altitude_monotonic = altitude.altitude_monotonic; // Altitud monotónica filtrada (en metros)

          antena_tracker_set_altura_del_drone(&antena_tracker, altitude_monotonic);

          // Serial.println();
          // Serial.println("*****----------*****");
          // Serial.println("Lee mensaje mavlink Altitude:  ");
          // Serial.print("altitude_monotonic:  ");
          // Serial.println(altitude_monotonic, 7);
          // Serial.print("altitude_amsl:  ");
          // Serial.println(altitude_amsl, 7);
          // Serial.println("*****----------*****");
          // Serial.println();
          // Serial.println();

          break;
        }

        default:
          printf("otro tipo de mensaje");
          break;
      }
    }

  }
}
