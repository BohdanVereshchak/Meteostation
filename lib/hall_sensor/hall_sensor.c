// hall_sensor.c
#include "hall_sensor.h"

static volatile uint32_t tip_count = 0;

uint32_t hall_sensor_get_tip_count(void) {
    return tip_count;
}

void hall_sensor_reset_count(void) {
    tip_count = 0;
}