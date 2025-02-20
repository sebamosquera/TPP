#ifndef I2C_H_
#define I2C_H_

#include "hardware/i2c.h"
#include <stdint.h>


#ifdef __cplusplus
extern "C"
{
#endif


int i2c_read_reg(i2c_inst_t *i2c_instance, uint8_t i2c_address, const uint8_t reg, uint8_t *buf, const size_t len);

int i2c_write(i2c_inst_t *i2c_instance, uint8_t i2c_address, const uint8_t data);

void i2c_write_u16_inline(i2c_inst_t *i2c_instance, uint8_t i2c_address, uint8_t reg, uint16_t value);

void i2c_write_bit_in_reg_inline(i2c_inst_t *i2c_instance, uint8_t i2c_address, uint8_t reg, uint8_t pos, uint8_t state);


#ifdef __cplusplus
}
#endif
#endif
