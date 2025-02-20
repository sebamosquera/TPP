#include "I2C.h"

int i2c_read_reg(i2c_inst_t *i2c_instance, uint8_t i2c_address, const uint8_t reg, uint8_t *buf, const size_t len)
{
    i2c_write_blocking(i2c_instance, i2c_address, &reg, 1, true);
    return i2c_read_blocking(i2c_instance, i2c_address, buf, len, false);
}

int i2c_write(i2c_inst_t *i2c_instance, uint8_t i2c_address, const uint8_t data)
{
    return i2c_write_blocking(i2c_instance, i2c_address, &data, 1, false);
}

void i2c_write_u16_inline(i2c_inst_t *i2c_instance, uint8_t i2c_address, uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    i2c_write_blocking(i2c_instance, i2c_address, data, 3, false);
}

void i2c_write_bit_in_reg_inline(i2c_inst_t *i2c_instance, uint8_t i2c_address, uint8_t reg, uint8_t pos, uint8_t state)
{
    uint8_t reg_value;
    i2c_read_reg(i2c_instance, i2c_address, reg, &reg_value, 1);

    if (state)
    {
        reg_value |= (1 << pos);
    }
    else
    {
        reg_value &= ~(1 << pos);
    }

    uint8_t data[2] = {reg, reg_value};
    i2c_write_blocking(i2c_instance, i2c_address, data, 2, false);
}
