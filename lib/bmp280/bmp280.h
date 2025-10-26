// bmp280.h
#pragma once
#include "driver/i2c.h"

void bmp280_init(i2c_port_t i2c_num);
void bmp280_read_data(i2c_port_t i2c_num, float *temperature, float *pressure, float *humidity);