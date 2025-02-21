#ifndef Magnetometer_h
#define Magnetometer_h

#include <stdint.h>
#include <stddef.h>

#include "config.h"
#include "I2C.h"



#ifdef __cplusplus
extern "C"
{
#endif



#define HMC5883L_ADDRESS               0x1E // this device only has one address
#define HMC5883L_DEFAULT_ADDRESS       0x1E

#define HMC5883L_REG_CONFIG_A          0x00
#define HMC5883L_REG_CONFIG_B          0x01
#define HMC5883L_REG_MODE              0x02
#define HMC5883L_REG_DATAX_H           0x03
#define HMC5883L_REG_DATAX_L           0x04
#define HMC5883L_REG_DATAZ_H           0x05
#define HMC5883L_REG_DATAZ_L           0x06
#define HMC5883L_REG_DATAY_H           0x07
#define HMC5883L_REG_DATAY_L           0x08
#define HMC5883L_REG_STATUS            0x09
#define HMC5883L_REG_ID_A              0x0A
#define HMC5883L_REG_ID_B              0x0B
#define HMC5883L_REG_ID_C              0x0C

#define HMC5883L_CRA_AVERAGE_BIT    6
#define HMC5883L_CRA_AVERAGE_LENGTH 2
#define HMC5883L_CRA_RATE_BIT       4
#define HMC5883L_CRA_RATE_LENGTH    3
#define HMC5883L_CRA_BIAS_BIT       1
#define HMC5883L_CRA_BIAS_LENGTH    2

enum MAGNETOMETRO_AVERAGING
{
    HMC5883L_SAMPLES_1 = 0,
    HMC5883L_SAMPLES_2 = 1,
    HMC5883L_SAMPLES_4 = 2,
    HMC5883L_SAMPLES_8 = 3,
};

enum MAGNETOMETRO_RATE
{
    HMC5883L_DATARATE_0_75_HZ = 0,
    HMC5883L_DATARATE_1_5_HZ  = 1,
    HMC5883L_DATARATE_3_HZ    = 2,
    HMC5883L_DATARATE_7_5_HZ  = 3,
    HMC5883L_DATARATE_15_HZ   = 4,
    HMC5883L_DATARATE_30_HZ   = 5,
    HMC5883L_DATARATE_75_HZ   = 6,
};

enum MAGNETOMETRO_BIAS
{
    HMC5883L_BIAS_NORMAL      = 0,
    HMC5883L_BIAS_POSITIVE    = 1,
    HMC5883L_BIAS_NEGATIVE    = 2,
};

#define HMC5883L_CRB_GAIN_BIT       7
#define HMC5883L_CRB_GAIN_LENGTH    3

enum MAGNETOMETRO_RANGE
{
    HMC5883L_RANGE_0_8GA = 0,
    HMC5883L_RANGE_1_3GA = 1,
    HMC5883L_RANGE_1_9GA = 2,
    HMC5883L_RANGE_2_5GA = 3,
    HMC5883L_RANGE_4GA   = 4,
    HMC5883L_RANGE_4_7GA = 5,
    HMC5883L_RANGE_5_6GA = 6,
    HMC5883L_RANGE_8_1GA = 7,
};

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

enum MAGNETOMETRO_MODE
{
    HMC5883L_MODE_CONTINUOUS = 0,
    HMC5883L_MODE_SINGLE     = 1,
    HMC5883L_MODE_IDLE       = 2,
};

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0


// Valores hardcodeados, mejorar
#define MAGNETO_SCALE_X 180.0
#define MAGNETO_SCALE_Y 200.0
// Valores hardcodeados, mejorar


typedef struct magnetometer
{
    struct i2c_information i2c;
    // struct magnetometer_configuration config;
    int16_vector_t magnet_offsets;
    bool calibrated;

    uint8_t mode;
} magnetometer_t;

bool magnetometer_init(magnetometer_t* self, i2c_inst_t *i2c_instance);

bool magnetometer_connected(magnetometer_t* self);

bool magnetometer_read_raw(magnetometer_t* self, int16_t *x, int16_t *y, int16_t *z);

bool magnetometer_read(magnetometer_t* self, float_vector_t *mag);



void magnetometer_set_magnet_offsets(magnetometer_t *self, int16_t *magnet_offsets);

void magnetometer_get_magnet_offsets(magnetometer_t *self, int16_t *magnet_offsets);

// CONFIG_A register
void magnetometer_set_averaging(magnetometer_t* self, enum MAGNETOMETRO_AVERAGING averaging);
void magnetometer_set_rate(magnetometer_t *self, enum MAGNETOMETRO_RATE rate);
void magnetometer_set_bias(magnetometer_t *self, enum MAGNETOMETRO_BIAS bias);

// CONFIG_B register
void magnetometer_set_range(magnetometer_t *self, enum MAGNETOMETRO_RANGE range);

// MODE register
void magnetometer_set_mode(magnetometer_t *self, enum MAGNETOMETRO_MODE mode);

#ifdef __cplusplus
}
#endif
#endif
