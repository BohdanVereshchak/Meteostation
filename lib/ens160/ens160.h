// ens160.h
#pragma once
#include "driver/i2c.h"

bool ens160_init(i2c_port_t i2c_num);
bool ens160_read_data(i2c_port_t i2c_num, uint16_t *aqi, uint16_t *tvoc, uint16_t *eco2);