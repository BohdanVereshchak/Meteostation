#include "bmp280.h"
#include "driver/i2c.h"
#include <math.h>
#include "esp_log.h"

#define BMP280_ADDRESS 0x77
#define AHT20_ADDRESS 0x38
#define BMP280_REG_ID 0xD0
#define BMP280_REG_CTRL 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7

static int32_t t_fine;
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static uint8_t bmp280_read_byte(i2c_port_t i2c_num, uint8_t reg) {
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return data;
}

static void bmp280_read_bytes(i2c_port_t i2c_num, uint8_t reg, uint8_t *data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

static void bmp280_write_byte(i2c_port_t i2c_num, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

static void bmp280_read_calibration_data(i2c_port_t i2c_num) {
    uint8_t data[24];
    bmp280_read_bytes(i2c_num, 0x88, data, 24);

    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];

    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11] << 8) | data[10];
    dig_P4 = (data[13] << 8) | data[12];
    dig_P5 = (data[15] << 8) | data[14];
    dig_P6 = (data[17] << 8) | data[16];
    dig_P7 = (data[19] << 8) | data[18];
    dig_P8 = (data[21] << 8) | data[20];
    dig_P9 = (data[23] << 8) | data[22];
}

void bmp280_init(i2c_port_t i2c_num) {
    uint8_t id = bmp280_read_byte(i2c_num, BMP280_REG_ID);
    if (id != 0x58) {
        printf("BMP280 not found, ID: 0x%02x\n", id);
        return;
    }

    bmp280_read_calibration_data(i2c_num);
    bmp280_write_byte(i2c_num, BMP280_REG_CTRL, 0x3F);
    bmp280_write_byte(i2c_num, BMP280_REG_CONFIG, 0x10);
}

static float bmp280_compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
              ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return (float)T / 100.0f;
}

static float bmp280_compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 += (var1 * (int64_t)dig_P5) << 17;
    var2 += ((int64_t)dig_P4) << 35;
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)dig_P1)) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (float)p / 256.0f;
}

float aht20_read_humidity(i2c_port_t i2c_num) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xAC, true);
    i2c_master_write_byte(cmd, 0x33, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    vTaskDelay(pdMS_TO_TICKS(80));

    uint8_t data[7];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT20_ADDRESS << 1) | I2C_MASTER_READ, true);
    for (int i = 0; i < 6; i++)
        i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[6], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    uint32_t hum_raw = ((uint32_t)(data[1]) << 12) |
                       ((uint32_t)(data[2]) << 4) |
                       ((data[3] & 0xF0) >> 4);
    return ((float)hum_raw / 1048576.0f) * 100.0f;
}

void bmp280_read_data(i2c_port_t i2c_num, float *temperature, float *pressure, float *humidity) {
    uint8_t data[6];
    esp_err_t err = i2c_master_read_from_device(i2c_num, BMP280_ADDRESS, data, 6, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE("BMP280", "Failed to read pressure/temperature: %s", esp_err_to_name(err));
        *temperature = NAN;
        *pressure = NAN;
        *humidity = NAN;
        return;
    }


    bmp280_read_bytes(i2c_num, BMP280_REG_PRESS_MSB, data, 6);
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    *temperature = bmp280_compensate_T(adc_T);
    *pressure = bmp280_compensate_P(adc_P) / 100.0f;
    *humidity = aht20_read_humidity(i2c_num);
}