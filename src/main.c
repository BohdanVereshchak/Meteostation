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
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "bmp280.h"
#include "ens160.h"
#include "hall_sensor.h"
#include "mbedtls/md.h"

// WiFi and server configuration
#define WIFI_SSID "Murovane ABIV" //S B N V
#define WIFI_PASS "19801980"
#define SERVER_URL "https://meteostation-230224899515.europe-central2.run.app" //http://192.168.0.102:3000/api/data
#define SERVER_URL_MAX_LEN 128
char server_url[SERVER_URL_MAX_LEN] = SERVER_URL;

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

// Config portal variables
static bool config_portal_active = false;
static SemaphoreHandle_t config_saved_sem = NULL;
static const char config_page_html[] =
    "<!doctype html>\n"
    "<html><head><meta charset='utf-8'><title>Meteo config</title></head>\n"
    "<body>\n"
    "<h2>Meteostation configuration</h2>\n"
    "<label>WiFi SSID: <input id='ssid' value=''></label><br>\n"
    "<label>WiFi Password: <input id='pass' value='' type='password'></label><br>\n"
    "<label>Server URL: <input id='server' value=''></label><br>\n"
    "<label>Update interval ms: <input id='interval' value=''></label><br>\n"
    "<button onclick='save()'>Save & Reboot</button>\n"
    "<script>\n"
    "async function save(){\n"
    "  const body = { wifi_ssid: document.getElementById('ssid').value,\n"
    "                 wifi_password: document.getElementById('pass').value,\n"
    "                 server_url: document.getElementById('server').value,\n"
    "                 update_interval_ms: parseInt(document.getElementById('interval').value) || 30000 };\n"
    "  const resp = await fetch('/save', { method: 'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)});\n"
    "  const t = await resp.text();\n"
    "  alert(t);\n"
    "}\n"
    "fetch('/values').then(r=>r.json()).then(j=>{\n"
    " document.getElementById('ssid').value = j.wifi_ssid || '';\n"
    " document.getElementById('pass').value = j.wifi_password || '';\n"
    " document.getElementById('server').value = j.server_url || '';\n" "document.getElementById('interval').value = j.update_interval_ms || 30000;\n"
    "});\n"
    "</script>\n</body></html>";

// HTTP server handle
static httpd_handle_t config_httpd = NULL;

// Structures
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
"MIIDejCCAmKgAwIBAgIQf+UwvzMTQ77dghYQST2KGzANBgkqhkiG9w0BAQsFADBX\n"
"MQswCQYDVQQGEwJCRTEZMBcGA1UEChMQR2xvYmFsU2lnbiBudi1zYTEQMA4GA1UE\n"
"CxMHUm9vdCBDQTEbMBkGA1UEAxMSR2xvYmFsU2lnbiBSb290IENBMB4XDTIzMTEx\n"
"NTAzNDMyMVoXDTI4MDEyODAwMDA0MlowRzELMAkGA1UEBhMCVVMxIjAgBgNVBAoT\n"
"GUdvb2dsZSBUcnVzdCBTZXJ2aWNlcyBMTEMxFDASBgNVBAMTC0dUUyBSb290IFI0\n"
"MHYwEAYHKoZIzj0CAQYFK4EEACIDYgAE83Rzp2iLYK5DuDXFgTB7S0md+8Fhzube\n"
"Rr1r1WEYNa5A3XP3iZEwWus87oV8okB2O6nGuEfYKueSkWpz6bFyOZ8pn6KY019e\n"
"WIZlD6GEZQbR3IvJx3PIjGov5cSr0R2Ko4H/MIH8MA4GA1UdDwEB/wQEAwIBhjAd\n"
"BgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwDwYDVR0TAQH/BAUwAwEB/zAd\n"
"BgNVHQ4EFgQUgEzW63T/STaj1dj8tT7FavCUHYwwHwYDVR0jBBgwFoAUYHtmGkUN\n"
"l8qJUC99BM00qP/8/UswNgYIKwYBBQUHAQEEKjAoMCYGCCsGAQUFBzAChhpodHRw\n"
"Oi8vaS5wa2kuZ29vZy9nc3IxLmNydDAtBgNVHR8EJjAkMCKgIKAehhxodHRwOi8v\n"
"Yy5wa2kuZ29vZy9yL2dzcjEuY3JsMBMGA1UdIAQMMAowCAYGZ4EMAQIBMA0GCSqG\n"
"SIb3DQEBCwUAA4IBAQAYQrsPBtYDh5bjP2OBDwmkoWhIDDkic574y04tfzHpn+cJ\n"
"odI2D4SseesQ6bDrarZ7C30ddLibZatoKiws3UL9xnELz4ct92vID24FfVbiI1hY\n"
"+SW6FoVHkNeWIP0GCbaM4C6uVdF5dTUsMVs/ZbzNnIdCp5Gxmx5ejvEau8otR/Cs\n"
"kGN+hr/W5GvT1tMBjgWKZ1i4//emhA1JG1BbPzoLJQvyEotc03lXjTaCzv8mEbep\n"
"8RqZ7a2CPsgRbuvTPBwcOMBBmuFeU88+FSBX6+7iP0il8b4Z0QFqIwwMHfs/L6K1\n"
"vepuoxtGzi4CZ68zJpiq1UvSqTbFJjtbD4seiMHl\n"
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

// Install config into NVS in wifi_ssid, wifi_password, server_url
static void load_config_from_nvs(void) {
    nvs_handle_t nvs_handle;
    if (nvs_open("storage", NVS_READWRITE, &nvs_handle) != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed, using defaults");
        return;
    }

    uint32_t u32;
    if (nvs_get_u32(nvs_handle, "update_interval", &u32) == ESP_OK) {
        update_interval_ms = u32;
    }

    if (nvs_get_u32(nvs_handle, "tip_count", &u32) == ESP_OK) {
        tip_count = u32;
    }

    size_t required = sizeof(wifi_ssid);
    if (nvs_get_str(nvs_handle, "wifi_ssid", wifi_ssid, &required) != ESP_OK) {}

    required = sizeof(wifi_password);
    if (nvs_get_str(nvs_handle, "wifi_password", wifi_password, &required) != ESP_OK) {}

    required = sizeof(server_url);
    if (nvs_get_str(nvs_handle, "server_url", server_url, &required) != ESP_OK) {}

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Loaded config from NVS: ssid='%s', server_url='%s', interval=%u",
             wifi_ssid, server_url, update_interval_ms);
}

// Save config to NVS (wifi_ssid, wifi_password, server_url, update_interval_ms, tip_count)
static esp_err_t save_config_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed");
        return err;
    }

    err = nvs_set_u32(nvs_handle, "update_interval", update_interval_ms);
    if (err != ESP_OK) { nvs_close(nvs_handle); return err; }

    err = nvs_set_u32(nvs_handle, "tip_count", tip_count);
    if (err != ESP_OK) { nvs_close(nvs_handle); return err; }

    err = nvs_set_str(nvs_handle, "wifi_ssid", wifi_ssid);
    if (err != ESP_OK) { nvs_close(nvs_handle); return err; }

    err = nvs_set_str(nvs_handle, "wifi_password", wifi_password);
    if (err != ESP_OK) { nvs_close(nvs_handle); return err; }

    err = nvs_set_str(nvs_handle, "server_url", server_url);
    if (err != ESP_OK) { nvs_close(nvs_handle); return err; }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Configuration saved to NVS");
    return err;
}

static esp_err_t config_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, config_page_html, strlen(config_page_html));
    return ESP_OK;
}

static esp_err_t config_values_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "wifi_ssid", wifi_ssid);
    cJSON_AddStringToObject(root, "wifi_password", wifi_password);
    cJSON_AddStringToObject(root, "server_url", server_url);
    cJSON_AddNumberToObject(root, "update_interval_ms", update_interval_ms);
    char *s = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, s, strlen(s));
    free(s);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t config_save_handler(httpd_req_t *req) {
    int len = req->content_len;
    if (len <= 0 || len > 1024) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body");
        return ESP_FAIL;
    }
    char *buf = malloc(len + 1);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Alloc failed");
        return ESP_FAIL;
    }
    int ret = httpd_req_recv(req, buf, len);
    if (ret <= 0) {
        free(buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Recv failed");
        return ESP_FAIL;
    }
    buf[len] = '\0';
    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON parse error");
        return ESP_FAIL;
    }

    cJSON *jssid = cJSON_GetObjectItem(root, "wifi_ssid");
    cJSON *jpass = cJSON_GetObjectItem(root, "wifi_password");
    cJSON *jserver = cJSON_GetObjectItem(root, "server_url");
    cJSON *jinterval = cJSON_GetObjectItem(root, "update_interval_ms");

    if (jssid && cJSON_IsString(jssid)) {
        strncpy(wifi_ssid, jssid->valuestring, sizeof(wifi_ssid)-1);
        wifi_ssid[sizeof(wifi_ssid)-1] = '\0';
    }
    if (jpass && cJSON_IsString(jpass)) {
        strncpy(wifi_password, jpass->valuestring, sizeof(wifi_password)-1);
        wifi_password[sizeof(wifi_password)-1] = '\0';
    }
    if (jserver && cJSON_IsString(jserver)) {
        strncpy(server_url, jserver->valuestring, sizeof(server_url)-1);
        server_url[sizeof(server_url)-1] = '\0';
    }
    if (jinterval && cJSON_IsNumber(jinterval)) {
        update_interval_ms = jinterval->valueint;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t e = save_config_to_nvs();
    cJSON_Delete(root);

    if (e != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS save failed");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "Saved. Attempting to reconnect to new WiFi...");
    if (config_saved_sem) {
        xSemaphoreGive(config_saved_sem);
    }
    //esp_restart();
    return ESP_OK;
}

static esp_err_t start_config_portal(void) {
    if (config_portal_active) return ESP_OK;

    ESP_LOGI(TAG, "Starting config portal (AP + HTTP server)");

    //ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "MeteoConfig",
            .ssid_len = 0,
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    http_config.server_port = 80;
    if (httpd_start(&config_httpd, &http_config) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = config_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(config_httpd, &root);

        httpd_uri_t values = {
            .uri = "/values",
            .method = HTTP_GET,
            .handler = config_values_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(config_httpd, &values);

        httpd_uri_t save = {
            .uri = "/save",
            .method = HTTP_POST,
            .handler = config_save_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(config_httpd, &save);

        config_portal_active = true;
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server for config portal");
        return ESP_FAIL;
    }
}

static void stop_config_portal(void) {
    if (!config_portal_active) return;
    if (config_httpd) {
        httpd_stop(config_httpd);
        config_httpd = NULL;
    }
    // повернутися в STA режим (не перезавантажуємо тут)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    config_portal_active = false;
    ESP_LOGI(TAG, "Config portal stopped; wifi set to STA");
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    static uint32_t backoff_ms = 10000;
    static const uint32_t BACKOFF_MAX_MS = 60000;
    static int64_t last_attempt_ms = 0;

    int64_t now_ms = esp_timer_get_time() / 1000;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (!config_portal_active) {
            ESP_LOGI(TAG, "WIFI_EVENT_STA_START: attempting connect");
            esp_wifi_connect();
            last_attempt_ms = now_ms;
            backoff_ms = 10000; // скидаємо початковий backoff
        } else {
            ESP_LOGI(TAG, "WIFI_EVENT_STA_START: config portal active, skipping auto-connect");
        }

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *dis = (wifi_event_sta_disconnected_t*) event_data;
        int reason = dis ? dis->reason : -1;
        wifi_connected = false;
        if (config_portal_active) {
            ESP_LOGI(TAG, "STA disconnected (reason %d) but config portal active => skipping reconnect", reason);
            return;
        }
        ESP_LOGW(TAG, "STA disconnected (reason %d). Last attempt %lld ms ago. Backoff %u ms",
                 reason, (long long)(now_ms - last_attempt_ms), backoff_ms);
        if ((now_ms - last_attempt_ms) >= (int64_t)backoff_ms) {
            esp_err_t r = esp_wifi_connect();
            if (r == ESP_OK) {
                last_attempt_ms = now_ms;
                ESP_LOGI(TAG, "esp_wifi_connect() initiated");
            } else {
                backoff_ms = backoff_ms * 2;
                if (backoff_ms > BACKOFF_MAX_MS) backoff_ms = BACKOFF_MAX_MS;
                last_attempt_ms = now_ms;
                ESP_LOGW(TAG, "esp_wifi_connect() failed: %s — increasing backoff to %u ms", esp_err_to_name(r), backoff_ms);
            }
        } else {
            ESP_LOGI(TAG, "Skipping reconnect attempt; will retry after backoff");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        backoff_ms = 10000;
        last_attempt_ms = now_ms;
    }
}

static void wifi_task(void *pvParameters) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = { 0 };

    strncpy((char*)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid)-1);
    strncpy((char*)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password)-1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(esp_wifi_start());

    int fail_count = 0;
    const int MAX_FAILS_BEFORE_PORTAL = 2;

    while (1) {
        if (!wifi_connected) {
            ESP_LOGI(TAG, "WiFi not connected, attempting to reconnect...");
            esp_wifi_connect();
            fail_count++;
            vTaskDelay(10000 / portTICK_PERIOD_MS); // Wait 10 seconds before retry
            if (fail_count >= MAX_FAILS_BEFORE_PORTAL) {
                ESP_LOGW(TAG, "Unable to connect after %d attempts, starting config portal", fail_count);
                if (start_config_portal() != ESP_OK) {
                    ESP_LOGE(TAG, "start_config_portal failed");
                    vTaskDelay(30000 / portTICK_PERIOD_MS);
                    fail_count = 0;
                    continue;
                }

                if (config_saved_sem) {
                    ESP_LOGI(TAG, "Waiting for user to save new config...");
                    if (xSemaphoreTake(config_saved_sem, portMAX_DELAY) == pdTRUE) {
                        ESP_LOGI(TAG, "Config saved by user, proceeding to live reconnect");
                    } else {
                        ESP_LOGW(TAG, "Timeout waiting for saved config");
                    }
                }
                stop_config_portal();

                wifi_config_t new_conf = { 0 };
                strncpy((char*)new_conf.sta.ssid, wifi_ssid, sizeof(new_conf.sta.ssid)-1);
                strncpy((char*)new_conf.sta.password, wifi_password, sizeof(new_conf.sta.password)-1);
                new_conf.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

                esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA); // ensure STA
                if (err != ESP_OK) ESP_LOGW(TAG, "esp_wifi_set_mode STA failed: %s", esp_err_to_name(err));

                err = esp_wifi_set_config(ESP_IF_WIFI_STA, &new_conf);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(err));
                } else {
                    esp_wifi_connect();
                }
                fail_count = 0;
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                continue;
            }
        } else {
            vTaskDelay(600000 / portTICK_PERIOD_MS); // Check connection every 10 minute
        }
    }
    vTaskDelete(NULL);
}

// HTTP GET request to receive JSON config data
esp_err_t get_config_from_server() {
    if (!wifi_connected) {
        ESP_LOGE(TAG, "Cannot get config - WiFi not connected");
        return ESP_FAIL;
    }

    char cfg_url[SERVER_URL_MAX_LEN + 16];
    snprintf(cfg_url, sizeof(cfg_url), "%s/api/config", server_url);

    esp_http_client_config_t config = {
        .url = cfg_url,
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
        if (!config_portal_active) {
            if (start_config_portal() == ESP_OK) {
                ESP_LOGI(TAG, "Local config portal active at station IP");
                ESP_LOGI(TAG, "Connect to http://<ESP_IP>/ to update server URL");
            }
        }
        return err;
    } else {
        content_length = esp_http_client_fetch_headers(client);
        if (content_length < 0) {
            ESP_LOGE(TAG, "HTTP client fetch headers failed");
        } else {
            int data_read = esp_http_client_read_response(client, buffer, 512);
            if (data_read >= 0) {
                buffer[data_read] = '\0'; // Null-terminate the buffer
                int status = esp_http_client_get_status_code(client);
                ESP_LOGI(TAG, "HTTP GET Status = %d, Response: %s", status, buffer);
                if (status == 200 || status == 204) {
                    if (config_portal_active) {
                        stop_config_portal();
                        config_portal_active = false;
                        ESP_LOGI(TAG, "Server connection successful — config portal stopped");
                    }
                }
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

// HTTPS POST request to send JSON data
static esp_err_t send_data_to_server_secure(const char *json_data) {
    char full_url[160];
    snprintf(full_url, sizeof(full_url), "%s%s", server_url, "/api/data");

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

    char cfg_url[SERVER_URL_MAX_LEN + 16];
    snprintf(cfg_url, sizeof(cfg_url), "%s/time", server_url);

    esp_http_client_config_t config = {
        .url = cfg_url,
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
    esp_err_t err;
    do {
        err = sync_time_with_retry(3);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Initial time sync failed, retrying in 120s...");
            vTaskDelay(pdMS_TO_TICKS(120000));
        }
    } while (err != ESP_OK);

    ESP_LOGI(TAG, "Initial time sync successful");

    time_t current_unix = time_tracker.base_unix_time;
    time_t last_sync_time = current_unix;

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

        if (current_unix - last_sync_time >= 360000) {
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
                vTaskDelay(pdMS_TO_TICKS(60000));
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

    config_saved_sem = xSemaphoreCreateBinary();
    if (config_saved_sem == NULL) {
        ESP_LOGW(TAG, "Failed to create config_saved_sem");
    }
    load_config_from_nvs();
    time_tracker.mutex = xSemaphoreCreateMutex();

    printf("Initializing I2C interfaces...\n");
    init_i2c();
    
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