#ifndef I2C_H
#define I2C_H

#include <Arduino.h>

#define I2C_ADD      0x14
#define I2C_SDA_PIN  22
#define I2C_SCL_PIN  21
#define I2C_INT_PIN  32
#define I2C_RST_PIN  13

void i2c_init();
void i2c_aux_mode(bool aux_on);
void i2c_prepare_touch(uint16_t x, uint16_t y);
void i2c_release_touch();   /* V1.1: nhả ngón tay ngay khi nhận ACTION_UP */
void i2c_touch_tick();

#endif