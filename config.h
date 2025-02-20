#ifndef Config_h
#define Config_h

#include "hardware/i2c.h"


#define LED_PIN 25

// ----------------- Configuraciones comunicaciones
#define UART_INSTANCE_ uart0
#define PIN_TX 0
#define PIN_RX 1

#define I2C_INSTANCE_MAGNET i2c1
#define I2C_INSTANCE_MPU i2c0

#define PIN_MAGNET_SDA 2 // blanco
#define PIN_MAGNET_SCL 3 // naranja

#define PIN_MPU_SDA 20 // Violeta
#define PIN_MPU_SCL 21 // Amarillo
// ----------------- Configuraciones comunicaciones


// ----------------- Configuraciones motores
#define VELOCIDAD_MOTOR 10 // 0.05
#define PASOS_POR_VUELTA 200

#define PIN_PWM_1 16
#define PIN_DIR_1 17

#define PIN_PWM_2 14
#define PIN_DIR_2 15

// Ratios de poleas de motores
#define RATIO_AZIMUTH 4.55
#define RATIO_ELEVACION 4
// ----------------- Configuraciones motores


// ----------  ESTRUCTURAS DE DATOS (CAMBIAR DE ARCHIVO)
struct i2c_information
{
    i2c_inst_t *instance;
    uint8_t address;
};

typedef struct int16_vector {
    int16_t x;
    int16_t y;
    int16_t z;
} int16_vector_t;

typedef struct float_vector {
    float x;
    float y;
    float z;
} float_vector_t;
// ----------  ESTRUCTURAS DE DATOS (CAMBIAR DE ARCHIVO)

#endif
