#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/PCA9685.hpp"
#include "MotionController.hpp"
#include "UartHandler.hpp"
#include <memory>
#include <i2cdev.h>
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "config.h"
#include "esp_http_server.h"
#include "json_parser.h"

static const char *TAG = "MAIN";

static std::unique_ptr<MotionController> motion_controller_ptr;

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

// 新的调参API处理器
static esp_err_t tuning_api_handler(httpd_req_t *req)
{
    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {  // 错误或连接关闭
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[ret] = '\0';
    ESP_LOGI(TAG, "Received tuning data: %s", content);

    jparse_ctx_t jctx;
    json_parse_start(&jctx, content, ret);

    char cmd_buf[32];
    if (json_obj_get_string(&jctx, "command", cmd_buf, sizeof(cmd_buf)) != 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'command' in JSON");
        ESP_LOGE(TAG, "Invalid JSON format or missing 'command'");
        return ESP_FAIL;
    }

    std::string command(cmd_buf);

    if (command == "get_params") {
        char action_buf[64];
        if (json_obj_get_string(&jctx, "action", action_buf, sizeof(action_buf)) == 0) {
            ESP_LOGI(TAG, "Getting action params for: %s", action_buf);
            std::string json_response = motion_controller_ptr->get_action_params_json(action_buf);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, json_response.c_str(), json_response.length());
            ESP_LOGI(TAG, "Sent action params for '%s': %s", action_buf, json_response.c_str());
            return ESP_OK;
        }
    } else if (command == "tune_param") {
        char action_buf[64], param_type_buf[32];
        int servo_index;
        float value;
        if (json_obj_get_string(&jctx, "action", action_buf, sizeof(action_buf)) == 0 &&
            json_obj_get_int(&jctx, "servo", &servo_index) == 0 &&
            json_obj_get_string(&jctx, "param", param_type_buf, sizeof(param_type_buf)) == 0 &&
            json_obj_get_float(&jctx, "value", &value) == 0) {
            
            motion_controller_ptr->tune_gait_parameter(action_buf, servo_index, param_type_buf, (float)value);
            httpd_resp_send(req, "Tune OK", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    } else if (command == "save_params") {
        char action_buf[64];
        if (json_obj_get_string(&jctx, "action", action_buf, sizeof(action_buf)) == 0) {
            motion_controller_ptr->save_action_to_nvs(action_buf);
            httpd_resp_send(req, "Save OK", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON format or command");
    return ESP_FAIL;
}

// 旧的API处理器保持不变
static esp_err_t command_api_handler(httpd_req_t *req)
{
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    char param_val[32];
    if (httpd_query_key_value(buf, "motion", param_val, sizeof(param_val)) == ESP_OK) {
        motion_command_t cmd = { .motion_type = (uint8_t)atoi(param_val), .param = 0 };
        motion_controller_ptr->queue_command(cmd);
        httpd_resp_send(req, "Command Queued", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static esp_err_t root_handler(httpd_req_t *req)
{
    const uint32_t index_html_len = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html_start, index_html_len);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = root_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t control_uri = {
            .uri      = "/control",
            .method   = HTTP_POST,
            .handler  = command_api_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &control_uri);
        
        // 注册新的调参API
        httpd_uri_t tune_uri = {
            .uri       = "/api/tune",
            .method    = HTTP_POST,
            .handler   = tuning_api_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &tune_uri);
        
        return server;
    }
    return NULL;
}

// --- Wi-Fi and app_main remain the same ---

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Retrying to connect to the AP");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        start_webserver();
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t instance_any_id, instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));
    wifi_config_t wifi_config = { };
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASSWORD);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Application startup.");
    wifi_init();
    i2cdev_init();

    PCA9685 servo_driver;
    servo_driver.init();

    motion_controller_ptr = std::make_unique<MotionController>(servo_driver);
    motion_controller_ptr->init();

    auto uart_handler_ptr = std::make_unique<UartHandler>(*motion_controller_ptr);
    uart_handler_ptr->init();
    
    ESP_LOGI(TAG, "Initialization complete. All modules started.");

    // Clean and re-register default actions for debugging
    // motion_controller_ptr->delete_action_from_nvs("walk_forward");
    // motion_controller_ptr->delete_action_from_nvs("walk_backward");
    // motion_controller_ptr->delete_action_from_nvs("turn_left");
    // motion_controller_ptr->delete_action_from_nvs("turn_right");
    // motion_controller_ptr->register_default_actions();

    while (true) {
        motion_controller_ptr->servo_test(15, 90); // 15通道总是用来校准舵机，方便机械安装
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}