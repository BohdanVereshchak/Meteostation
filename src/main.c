#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include "bmp280.h"
#include "ens160.h"
#include "hall_sensor.h"

// WiFi and server configuration
#define WIFI_SSID "Murovane ABIV" //S B N V
#define WIFI_PASS "19801980"
#define SERVER_URL "https://192.168.0.102:3000" //http://192.168.0.102:3000/api/data
static const char *TAG = "meteostation";

// Configuration
#define DEFAULT_UPDATE_INTERVAL_MS 30000       // 5 minutes

// GPIO Definitions
#define I2C_SCL_GPIO 22 // BMP280 SCL
#define I2C_SDA_GPIO 21 // BMP280 SDA
#define I2C_SDA_GPIO_2 18 // ENS160 SDA
#define I2C_SCL_GPIO_2 19 // ENS160 SCL
#define HALL_SENSOR_GPIO 15

// Task priorities
#define WIFI_TASK_PRIORITY 2
#define SENSOR_TASK_PRIORITY 4
#define TIME_TASK_PRIORITY 3
#define RAIN_TASK_PRIORITY 5

// Global variables
static QueueHandle_t hall_event_queue = NULL;
static QueueHandle_t sensor_data_queue = NULL;
static volatile uint32_t tip_count = 0;
static uint32_t last_tip_time = 0;
static bool wifi_connected = false;
static uint32_t update_interval_ms = DEFAULT_UPDATE_INTERVAL_MS;
uint32_t last_interval;
char wifi_ssid[64] = WIFI_SSID;
char wifi_password[64] = WIFI_PASS;


typedef struct {
    time_t base_unix_time;
    int64_t base_esp_time_us;
    char current_datetime[32];
    SemaphoreHandle_t mutex;
} time_tracker_t;

typedef struct {
    float temperature;
    float pressure;
    float humidity;
    uint16_t aqi;
    uint16_t tvoc;
    uint16_t eco2;
    uint32_t tips;
    time_t last_tip_time;
} sensor_data_t;

time_tracker_t time_tracker;
static sensor_data_t sensor_data = {0};

static const char server_root_cert_pem[] = 
"-----BEGIN CERTIFICATE-----\n"
"MIIFETCCAvmgAwIBAgIUcRMdE/6KXSUqF+cIYeFBjb8d1jQwDQYJKoZIhvcNAQEL\n"
"BQAwGDEWMBQGA1UEAwwNTXlMb2NhbFJvb3RDQTAeFw0yNTEwMjcxNzIzMDdaFw0y\n"
"ODA4MTYxNzIzMDdaMBgxFjAUBgNVBAMMDU15TG9jYWxSb290Q0EwggIiMA0GCSqG\n"
"SIb3DQEBAQUAA4ICDwAwggIKAoICAQC1yK8eeUpd0kmDXU6HGkudObaKft9EudtO\n"
"KkrgXgb1nQd/1nIyoVn2Mog4s6aeTIJpGU7A6vGRQekpVU8z4XqRHRBUESfNid7s\n"
"A7f48voHkzWUTzVfdUETYau5cffLcE03LWxzA2lT0sUbXfr7D6gZmkvHx4OzT56l\n"
"g7VMhKWCsejADtHC7qbYW0CzZuwQUW18jK4RjnAoEPRMcJYtSgmOUVYwoMBCs8K7\n"
"ti9PFZ8xHpqaYNMlZBfQqTjM8HLWzoU98BtFxKI4rdjRW3xGejY8IdyFWtxHs6Hz\n"
"7GbbGG0FzrNm8Ec5L7nDjHjD/5uIElXbiB085KLvt1wzBf0vlwLj24CS9unGheZR\n"
"wbeAwTIa4FYPkeNKh7Z3i1eXjub2PdJJfkvc647wgfR4+QQTXbiAKBWyon/tTKEb\n"
"iSjWSAr8h7nZG0ALBOdcmre6QpsMnepB6MmsPk3EFoI8k+YkoZfQ1Fuc7hmvFGKQ\n"
"YAqUexxOoYc8grgRtluLDMx/hmSNkYKMOjMiUaf490mySuA4I+4Lr57U9hhZVMSX\n"
"H2e+JWgRG6SXZopvwA/nS5C56yMT+UXRoTvJCYAC6S2hHxc4EVFWChLbEkmjwSl8\n"
"+MNLPQrkQdh6waaDtBZMoKnPuJsPYNvm10rCk8bfab5U3dqhTgbaCZJOy/Bz3cAL\n"
"unCee0XVNwIDAQABo1MwUTAdBgNVHQ4EFgQUF3NXNk9QxktDYWs/bsUWey1p6D0w\n"
"HwYDVR0jBBgwFoAUF3NXNk9QxktDYWs/bsUWey1p6D0wDwYDVR0TAQH/BAUwAwEB\n"
"/zANBgkqhkiG9w0BAQsFAAOCAgEAFOdkoZznC1/+laWY3Q//wB1810HRZeGz5TvP\n"
"kYXiSeoL/Hw/xXzUwBj1yI5lfgBHwwCChobQ06D8DBErJ9dmr3nQrbPUTwle5gpp\n"
"S9fdex7UkDB51q/SBo6dWpaPNQpOdrfOCC6h1WuA1sDKomod7/roI3Ubefu6js4q\n"
"B2CMvAjxpPQdT9uya2CMeyp6QKELYmg/Yk2qrfYuSq+nG3XqhpGoJLLO7HXTvDf/\n"
"RxCeCDVpOUJcw5TJEoJ89N2ggnNGZdM/cQzfWU/Y9oa13i9paLxpUprYia3Tyadi\n"
"qc4rRek4YltBxBXUIrwFeIIcd+ylSpxm0VD1rKgH2G8P5o3KWmYDmfDSOrhtVX3w\n"
"kio55809LdrU5M8I3jU3LgIclcceB/Zq1EJBjmDDPjI5hcKLljYgO+PGOT1QVuq3\n"
"XapxNX/Hrlvvsn0TroxqLuhiIv5H92T7B4WWTFydsc2uf4/puVf2OoHe91QPIvgZ\n"
"8Vlv6yChdaigBAcX5vy5oIyEcHaqivYfOP+4uuMyeCwNX7yTaOgr0iD27v/Ydk5s\n"
"zn/ZocFqYjvAv/0qRqnAeJaGfh7a2gBti3wnmU6uce/LpVLz//OjhMftPN7uQcVz\n"
"dKBC28LZKRbjQ+aMXZK52Gt7R/RtT43oytYLl546GQwLkAfDgkxqMD18vaiQX2eN\n"
"sJfCTcY=\n"
"-----END CERTIFICATE-----\n";

static const char *DEVICE_TOKEN = "MeteostationVereshchakToken";
static const unsigned char HMAC_KEY[] = "BogdanNaziariiSimkoCharchok";

char *compute_hmac_hex(const char *payload) {
    if (!payload) return NULL;
    unsigned char output[32];
    const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!md_info) {
        ESP_LOGE(TAG, "mbedtls md info error");
        return NULL;
    }
    int ret = mbedtls_md_hmac(md_info, HMAC_KEY, sizeof(HMAC_KEY)-1,
                              (const unsigned char *)payload, strlen(payload), output);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_md_hmac failed: -0x%04x", -ret);
        return NULL;
    }
    // Convert to hex string
    char *hex = malloc(65);
    if (!hex) return NULL;
    for (int i = 0; i < 32; ++i) {
        sprintf(hex + i*2, "%02x", output[i]);
    }
    hex[64] = '\0';
    return hex;
}

// Some Function Declarations
esp_err_t sync_time_with_retry(int max_retries);

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying to connect to the WiFi");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
    }
}

static void wifi_task(void *pvParameters) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(esp_wifi_start());

    while (1) {
        if (!wifi_connected) {
            ESP_LOGI(TAG, "WiFi not connected, attempting to reconnect...");
            esp_wifi_connect();
            vTaskDelay(10000 / portTICK_PERIOD_MS); // Wait 10 seconds before retry
        } else {
            vTaskDelay(600000 / portTICK_PERIOD_MS); // Check connection every 10 minute
        }
    }
}

// HTTP GET request to receive JSON config data
esp_err_t get_config_from_server() {
    if (!wifi_connected) {
        ESP_LOGE(TAG, "Cannot get config - WiFi not connected");
        return ESP_FAIL;
    }

    esp_http_client_config_t config = {
        .url = SERVER_URL "/api/config",
        .timeout_ms = 5000,
        .cert_pem = server_root_cert_pem,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
    };

    char buffer[512+1]={0};
    int content_length = 0;

    ESP_LOGI(TAG, "HTTP native request =>");
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init http client");
        return ESP_FAIL;
    }

    // GET Request
    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_http_client_set_header(client, "Authorization", DEVICE_TOKEN);
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    } else {
        content_length = esp_http_client_fetch_headers(client);
        if (content_length < 0) {
            ESP_LOGE(TAG, "HTTP client fetch headers failed");
        } else {
            int data_read = esp_http_client_read_response(client, buffer, 512);
            if (data_read >= 0) {
                buffer[data_read] = '\0'; // Null-terminate the buffer
                ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %"PRId64,
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
                ESP_LOGI(TAG, "Response: %s", buffer);
            } else {
                ESP_LOGE(TAG, "Failed to read response");
            }
        }
    }
    esp_http_client_cleanup(client);

    // Parse JSON
    cJSON *root = cJSON_Parse(buffer);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return ESP_FAIL;
    }

    cJSON *system = cJSON_GetObjectItem(root, "system");
    if (system) {
        cJSON *interval = cJSON_GetObjectItem(system, "update_interval_ms");
        if (interval && cJSON_IsNumber(interval) && interval->valueint != update_interval_ms) {
            last_interval = update_interval_ms;
            update_interval_ms = interval->valueint;
            ESP_LOGI(TAG, "Updated interval: %d ms", update_interval_ms);
            nvs_handle_t nvs_handle;
            if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
                esp_err_t err;
                err = nvs_set_u32(nvs_handle, "update_interval", update_interval_ms);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set update_interval in NVS: %s", esp_err_to_name(err));
                }

                // Зберігаємо tip_count (tips)
                err = nvs_set_u32(nvs_handle, "tip_count", tip_count);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set tip_count in NVS: %s", esp_err_to_name(err));
                }

                // Підтверджуємо зміни в NVS
                err = nvs_commit(nvs_handle);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
                }
                nvs_close(nvs_handle);
            }
        } else {
            nvs_handle_t nvs_handle;
            if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
                esp_err_t err = nvs_set_u32(nvs_handle, "tip_count", tip_count);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set tip_count in NVS: %s", esp_err_to_name(err));
                }
                // Підтверджуємо зміни в NVS
                err = nvs_commit(nvs_handle);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
                }
                nvs_close(nvs_handle);
            }
            ESP_LOGI(TAG, "update_interval_ms not found, not a number or unchanged");
        }
    }

    cJSON *wifi = cJSON_GetObjectItem(root, "wifi");
    if (wifi) {
        cJSON *ssid = cJSON_GetObjectItem(wifi, "wifi_uid");
        if (ssid && cJSON_IsString(ssid) && ssid->valuestring != NULL && strlen(ssid->valuestring) > 0) {
            strncpy(wifi_ssid, ssid->valuestring, sizeof(wifi_ssid) - 1);
            wifi_ssid[sizeof(wifi_ssid) - 1] = '\0';
            ESP_LOGI(TAG, "Updated SSID: %s", wifi_ssid);
        }

        cJSON *pass = cJSON_GetObjectItem(wifi, "wifi_password");
        if (pass && cJSON_IsString(pass) && pass->valuestring != NULL && strlen(pass->valuestring) > 0) {

            strncpy(wifi_password, pass->valuestring, sizeof(wifi_password) - 1);
            wifi_password[sizeof(wifi_password) - 1] = '\0';
            ESP_LOGI(TAG, "Updated WiFi password");
        }
    }

    cJSON_Delete(root);;
    return ESP_OK;
}

// HTTP POST request to send JSON data
static esp_err_t send_data_to_server(const char *json_data) {
    char full_url[128];
    snprintf(full_url, sizeof(full_url), "%s%s", SERVER_URL, "/api/data");

    esp_http_client_config_t config = {
        .url = full_url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
        .cert_pem = NULL,
    };
    
    if (!wifi_connected) {
        ESP_LOGE(TAG, "Cannot send data - WiFi not connected");
        return ESP_FAIL;
    }
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init http client");
        return ESP_FAIL;
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_data, strlen(json_data));
    
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %ld, content_length = %ld",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(client);
    return err;
}
// HTTPS POST request to send JSON data
static esp_err_t send_data_to_server_secure(const char *json_data) {
    char full_url[160];
    snprintf(full_url, sizeof(full_url), "%s%s", SERVER_URL, "/api/data");

    esp_http_client_config_t config = {
        .url = full_url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
        .cert_pem = server_root_cert_pem, // важливо: сертифікат для перевірки TLS
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
    };

    if (!wifi_connected) {
        ESP_LOGE(TAG, "Cannot send data - WiFi not connected");
        return ESP_FAIL;
    }

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init http client");
        return ESP_FAIL;
    }

    // Compute HMAC signature for payload
    char *sig_hex = compute_hmac_hex(json_data);
    if (!sig_hex) {
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Authorization", DEVICE_TOKEN);
    esp_http_client_set_header(client, "X-Signature", sig_hex);
    esp_http_client_set_post_field(client, json_data, strlen(json_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        int content_len = esp_http_client_get_content_length(client);
        ESP_LOGI(TAG, "HTTPS POST Status = %d, content_length = %d", status, content_len);
        char buffer[256];
        int r = esp_http_client_read_response(client, buffer, sizeof(buffer)-1);
        if (r > 0) {
            buffer[r] = '\0';
            ESP_LOGI(TAG, "Response: %s", buffer);
        }
    } else {
        ESP_LOGE(TAG, "HTTPS POST request failed: %s", esp_err_to_name(err));
    }

    free(sig_hex);
    esp_http_client_cleanup(client);
    return err;
}

// Hall sensor interrupt handler
static void IRAM_ATTR hall_sensor_isr_handler(void *arg) {
    uint32_t now = esp_timer_get_time() / 1000;
    if ((now - last_tip_time) > 100) { // Debounce (100ms)
        tip_count++;
        last_tip_time = now;
        
        // Update time-based statistics
        time_t current_time = time(NULL);
        sensor_data.last_tip_time = current_time;
        
        xQueueSendFromISR(hall_event_queue, &now, NULL);
    }
}

// Initialize Hall sensor
static void init_hall_sensor(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HALL_SENSOR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);
    
    hall_event_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(HALL_SENSOR_GPIO, hall_sensor_isr_handler, NULL);
}

// Initialize I2C
static void init_i2c(void) {
    i2c_config_t conf0 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_config_t conf1 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO_2,
        .scl_io_num = I2C_SCL_GPIO_2,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf0));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf0.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C_NUM_0 initialized");

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &conf1));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, conf1.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C_NUM_1 initialized");

    vTaskDelay(100 / portTICK_PERIOD_MS);
}

esp_err_t fetch_time_from_server() {
    if (!wifi_connected) {
        ESP_LOGE(TAG, "Cannot fetch time - WiFi not connected");
        return ESP_FAIL;
    }

    esp_http_client_config_t config = {
        .url = SERVER_URL "/time",
        .timeout_ms = 5000,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .cert_pem = server_root_cert_pem
    };

    char buffer[512+1]={0};
    int content_length = 0;

    ESP_LOGI(TAG, "HTTP native request =>");
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init http client");
        return ESP_FAIL;
    }

    // GET Request
    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_http_client_set_header(client, "Authorization", DEVICE_TOKEN);
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    } else {
        content_length = esp_http_client_fetch_headers(client);
        if (content_length < 0) {
            ESP_LOGE(TAG, "HTTP client fetch headers failed");
        } else {
            int data_read = esp_http_client_read_response(client, buffer, 512);
            if (data_read >= 0) {
                buffer[data_read] = '\0'; // Null-terminate the buffer
                ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %"PRId64,
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
                ESP_LOGI(TAG, "Response: %s", buffer);
            } else {
                ESP_LOGE(TAG, "Failed to read response");
            }
        }
    }
    esp_http_client_cleanup(client);

    // Parse JSON
    cJSON *root = cJSON_Parse(buffer);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return ESP_FAIL;
    }

    cJSON *unix_time = cJSON_GetObjectItem(root, "unix_time");
    cJSON *iso_datetime = cJSON_GetObjectItem(root, "iso_datetime");

    if (!cJSON_IsNumber(unix_time) || !cJSON_IsString(iso_datetime)) {
        ESP_LOGE(TAG, "Invalid JSON fields");
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Unix time: %d", unix_time->valueint);
    ESP_LOGI(TAG, "ISO datetime: %s", iso_datetime->valuestring);

    // You can now set your system time here if needed, using unix_time->valueint
    struct timeval tv = {
        .tv_sec = (time_t)unix_time->valuedouble,
        .tv_usec = 0,
    };
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "System time updated.");
    if (time_tracker.mutex != NULL && xSemaphoreTake(time_tracker.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        time_tracker.base_unix_time = tv.tv_sec;
        time_tracker.base_esp_time_us = esp_timer_get_time();
        strncpy(time_tracker.current_datetime, iso_datetime->valuestring, sizeof(time_tracker.current_datetime) - 1);
        time_tracker.current_datetime[sizeof(time_tracker.current_datetime) - 1] = '\0'; // always null-terminate
        xSemaphoreGive(time_tracker.mutex);
    } else {
        ESP_LOGW(TAG, "Failed to take time_tracker mutex");
    }

    cJSON_Delete(root);;
    return ESP_OK;
}

esp_err_t sync_time_with_retry(int max_retries) {
    int retry = 0;
    while (retry < max_retries) {
        ESP_LOGI(TAG, "Time sync attempt %d/%d", retry+1, max_retries);
        esp_err_t err = fetch_time_from_server();
        
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Time synchronized successfully");
            
            // Verify the time is reasonable (after 2023)
            time_t current_time = time_tracker.base_unix_time + 
                                (esp_timer_get_time() - time_tracker.base_esp_time_us) / 1000000;
            if (current_time > 1672531200) { // January 1, 2023
                return err;
            } else {
                ESP_LOGW(TAG, "Received invalid time (before 2023), retrying...");
            }
        } else {
            ESP_LOGE(TAG, "Time sync failed: %s", esp_err_to_name(err));
        }
        
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        retry++;
    }
    ESP_LOGE(TAG, "Failed to sync time after %d attempts", max_retries);
    return ESP_FAIL;
}

void get_datetime_from_base(char *buffer, size_t len) {
    xSemaphoreTake(time_tracker.mutex, portMAX_DELAY);
    int64_t now_us = esp_timer_get_time();
    time_t current_unix = time_tracker.base_unix_time + (now_us - time_tracker.base_esp_time_us) / 1000000;
    struct tm now_tm;
    localtime_r(&current_unix, &now_tm);
    strftime(buffer, len, "%Y-%m-%d %H:%M:%S", &now_tm);
    xSemaphoreGive(time_tracker.mutex);
}

void time_updater_task(void *pvParameters) {
    while (!wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for WiFi connectio
    }
    esp_err_t err = sync_time_with_retry(3);
    printf("Time fetching - %s", esp_err_to_name(err));

    time_t current_unix = time_tracker.base_unix_time;
    static time_t last_sync_time = 0;
    if (err == ESP_OK) {
        last_sync_time = current_unix;
    }

    while (1) {
        int64_t now_us = esp_timer_get_time();
        current_unix = time_tracker.base_unix_time +
                            (now_us - time_tracker.base_esp_time_us) / 1000000;

        struct tm now_tm;
        localtime_r(&current_unix, &now_tm);

        char datetime_str[32];
        strftime(datetime_str, sizeof(datetime_str), "%Y-%m-%d %H:%M:%S", &now_tm);

        xSemaphoreTake(time_tracker.mutex, portMAX_DELAY);
        snprintf(time_tracker.current_datetime, sizeof(time_tracker.current_datetime), "%s", datetime_str);
        xSemaphoreGive(time_tracker.mutex);

        if (current_unix - last_sync_time >= 36000) {
            if (wifi_connected) {
                sync_time_with_retry(3);
                last_sync_time = current_unix;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // оновлюємо щосекунди
    }
}

// Create JSON data structure
static char* create_json_data(const sensor_data_t *data) {
    cJSON *root = cJSON_CreateObject();
    char timestamp[64];
    
    get_datetime_from_base(timestamp, sizeof(timestamp));
    
    cJSON_AddStringToObject(root, "timestamp", timestamp);
    
    // Environmental data
    cJSON *env = cJSON_CreateObject();
    cJSON_AddNumberToObject(env, "temperature", data->temperature);
    cJSON_AddNumberToObject(env, "pressure", data->pressure);
    cJSON_AddNumberToObject(env, "humidity", data->humidity);
    cJSON_AddItemToObject(root, "environment", env);
    
    // Air quality data
    cJSON *air = cJSON_CreateObject();
    cJSON_AddNumberToObject(air, "aqi", data->aqi);
    cJSON_AddNumberToObject(air, "tvoc", data->tvoc);
    cJSON_AddNumberToObject(air, "eco2", data->eco2);
    cJSON_AddItemToObject(root, "air_quality", air);
    
    // Rainfall data
    cJSON *rain = cJSON_CreateObject();
    cJSON_AddNumberToObject(rain, "tips", data->tips);
    cJSON_AddItemToObject(root, "rainfall", rain);
    
    // System info
    cJSON *system = cJSON_CreateObject();
    cJSON_AddNumberToObject(system, "update_interval_ms", update_interval_ms);
    cJSON_AddItemToObject(root, "system", system);
    
    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    return json_string;
}

// Sensor reading task
static void sensor_task(void *pvParameters) {
    ESP_LOGI(TAG, "Initializing sensors...");
    bmp280_init(I2C_NUM_0);
    ens160_init(I2C_NUM_1);

        while (1) {
            // Read sensor data
            bmp280_read_data(I2C_NUM_0, &sensor_data.temperature, &sensor_data.pressure, &sensor_data.humidity);
            ens160_read_data(I2C_NUM_1, &sensor_data.aqi, &sensor_data.tvoc, &sensor_data.eco2);
            
            // Update rain data
            sensor_data.tips = tip_count;
            
            // Send data to queue
            if (xQueueSend(sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send sensor data to queue");
            }

            // Get new config from server
            esp_err_t err = ESP_FAIL;
            if(wifi_connected)
                err = get_config_from_server();
            else {
                vTaskDelay(pdMS_TO_TICKS(10000));
                continue;
            }

            if (err == ESP_OK && update_interval_ms != last_interval) {
                last_interval = update_interval_ms;
                vTaskDelay(update_interval_ms / portTICK_PERIOD_MS);  // Новий інтервал відразу
                continue;
            }
            
            vTaskDelay(update_interval_ms / portTICK_PERIOD_MS);
        }
    }

// Data processing and transmission task
static void data_task(void *pvParameters) {
    while (1) {
        sensor_data_t data;
        if (xQueueReceive(sensor_data_queue, &data, portMAX_DELAY)){
            // Create JSON data
            char *json_data = create_json_data(&data);
            if (json_data == NULL) {
                ESP_LOGE(TAG, "Failed to create JSON data");
                continue;
            }

            vTaskDelay(pdMS_TO_TICKS(500)); // Wait for half a second before sending data

            // Send data to server
            esp_err_t ret = send_data_to_server_secure(json_data);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send data to server");
                // TODO: Store data locally for later transmission
            }
            
            ESP_LOGI(TAG, "Data sent: %s", json_data);
            free(json_data);

        }
    }
}

// Rain monitoring task
static void rain_task(void *pvParameters) {
    init_hall_sensor();
    
    while (1) {
        uint32_t queue_value;
        if (xQueueReceive(hall_event_queue, &queue_value, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Rain detected! Tipping count: %"PRIu32, tip_count);
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "\n\n=== Meteostation starting ===");
    ESP_LOGI(TAG, "Compiled on %s %s", __DATE__, __TIME__);

    // Timezone
    setenv("TZ", "UTC0", 1);
    tzset();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    time_tracker.mutex = xSemaphoreCreateMutex();

    printf("Initializing I2C interfaces...\n");
    init_i2c();
    
    // Load configuration from NVS
    esp_err_t err;
    nvs_handle_t nvs_handle;
    if (nvs_open("storage", NVS_READWRITE, &nvs_handle) == ESP_OK) {
        uint32_t tmp;
        err = nvs_get_u32(nvs_handle, "update_interval", &tmp);

        if (err != ESP_OK) {
            update_interval_ms = DEFAULT_UPDATE_INTERVAL_MS;  // дефолт
        }
        else {
            update_interval_ms = tmp;
        }

        err = nvs_get_u32(nvs_handle, "tip_count", &tmp);
        if (err != ESP_OK) {
            tip_count = 0; 
        }
        else {
            tip_count = tmp;
        }

        nvs_close(nvs_handle);
    }
    
    ESP_LOGI(TAG, "Configuration:");
    ESP_LOGI(TAG, "  Update interval: %d ms", update_interval_ms);
    ESP_LOGI(TAG, "Restored tip count from NVS: %"PRIu32, tip_count);
    last_interval = update_interval_ms;

    // Initialize rain statistics
    time_t current_time = time(NULL);
    memset(&sensor_data, 0, sizeof(sensor_data_t));  // Clear the struct
    sensor_data.tips = tip_count;
    sensor_data.last_tip_time = current_time;


    // Create queues
    sensor_data_queue = xQueueCreate(5, sizeof(sensor_data_t));
    if (sensor_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor data queue");
        return;
    }
    
    // Create tasks
    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, WIFI_TASK_PRIORITY, NULL);
    xTaskCreate(time_updater_task, "time_task", 4096, NULL, TIME_TASK_PRIORITY, NULL);
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, SENSOR_TASK_PRIORITY, NULL);
    xTaskCreate(data_task, "data_task", 4096, NULL, SENSOR_TASK_PRIORITY, NULL);
    xTaskCreate(rain_task, "rain_task", 2048, NULL, RAIN_TASK_PRIORITY, NULL);
    
    ESP_LOGI(TAG, "=== Meteostation started successfully ===\n");
}