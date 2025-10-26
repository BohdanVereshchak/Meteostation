#include "ens160.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

#define ENS160_I2C_ADDRESS 0x53

#define ENS160_REG_PART_ID     0x00
#define ENS160_REG_OPMODE      0x10
#define ENS160_REG_CONFIG      0x11
#define ENS160_REG_COMMAND     0x12
#define ENS160_REG_DATA_STATUS 0x20
#define ENS160_REG_AQI         0x21
#define ENS160_REG_TVOC        0x22
#define ENS160_REG_ECO2        0x24

static const char *TAG = "ENS160";

static esp_err_t ens160_write_byte(i2c_port_t i2c_num, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ENS160_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t ens160_read_bytes(i2c_port_t i2c_num, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ENS160_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ENS160_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

bool ens160_init(i2c_port_t i2c_num) {
    uint8_t part_id;
    if (ens160_read_bytes(i2c_num, ENS160_REG_PART_ID, &part_id, 1) != ESP_OK || part_id != 0x60) {
        ESP_LOGE(TAG, "ENS160 not found or wrong PART ID: 0x%02X", part_id);
        return false;
    }

    ens160_write_byte(i2c_num, ENS160_REG_OPMODE, 0x00); // Idle mode
    vTaskDelay(pdMS_TO_TICKS(10));

    ens160_write_byte(i2c_num, ENS160_REG_OPMODE, 0x02); // Standard mode
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "ENS160 initialized.");
    return true;
}

bool ens160_read_data(i2c_port_t i2c_num, uint16_t *aqi, uint16_t *tvoc, uint16_t *eco2) {
    uint8_t status;
    if (ens160_read_bytes(i2c_num, ENS160_REG_DATA_STATUS, &status, 1) != ESP_OK || !(status & 0x01)) {
        return false; // Data not ready
    }

    uint8_t data[5];
    if (ens160_read_bytes(i2c_num, ENS160_REG_AQI, data, 5) != ESP_OK) {
        *aqi = NAN;
        *tvoc = NAN;
        *eco2 = NAN;
        return false;
    }

    *aqi = data[0];
    *tvoc = data[1] | (data[2] << 8);
    *eco2 = data[3] | (data[4] << 8);
    return true;
}
