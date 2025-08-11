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

static const char *TAG = "MAIN";

static std::unique_ptr<MotionController> motion_controller_ptr;

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

static esp_err_t servo_api_handler(httpd_req_t *req)
{
    char* buf;
    size_t buf_len;

    // 获取 URL 查询字符串的长度
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "URL Query: %s", buf);
            char param_channel[16];
            char param_angle[16];

            // 解析 channel 和 angle 参数
            if (httpd_query_key_value(buf, "channel", param_channel, sizeof(param_channel)) == ESP_OK &&
                httpd_query_key_value(buf, "angle", param_angle, sizeof(param_angle)) == ESP_OK) {
                
                int channel = atoi(param_channel);
                int angle = atoi(param_angle);
                
                ESP_LOGI(TAG, "Received servo command: channel=%d, angle=%d", channel, angle);

                // 调用 MotionController 的方法
                if (motion_controller_ptr) {
                    motion_controller_ptr->servo_test(channel, angle);
                }
                
                free(buf);
                httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            }
        }
        free(buf);
    }
    
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

// Handler for queuing generic commands
static esp_err_t command_api_handler(httpd_req_t *req)
{
    char* buf;
    size_t buf_len;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "URL Query for command: %s", buf);
            char param_type[16];
            char param_param[16];

            if (httpd_query_key_value(buf, "type", param_type, sizeof(param_type)) == ESP_OK &&
                httpd_query_key_value(buf, "param", param_param, sizeof(param_param)) == ESP_OK) {
                
                motion_command_t cmd = {
                    .motion_type = (uint8_t)atoi(param_type),
                    .param = (uint8_t)atoi(param_param)
                };
                
                ESP_LOGI(TAG, "Received generic command: type=%d, param=%d", cmd.motion_type, cmd.param);

                if (motion_controller_ptr) {
                    if (motion_controller_ptr->queue_command(cmd)) {
                        free(buf);
                        httpd_resp_send(req, "Command queued successfully", HTTPD_RESP_USE_STRLEN);
                        return ESP_OK;
                    } else {
                        free(buf);
                        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to queue command");
                        return ESP_FAIL;
                    }
                }
            }
        }
        free(buf);
    }
    
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request: Missing type or param");
    return ESP_FAIL;
}

static esp_err_t root_handler(httpd_req_t *req)
{
    const uint32_t index_html_len = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html_start, index_html_len);
    return ESP_OK;
}

// 启动 Web 服务器
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // 注册 URI 处理函数
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);
        
        httpd_uri_t api_uri = {
            .uri       = "/api/servo",
            .method    = HTTP_GET,
            .handler   = servo_api_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_uri);

        httpd_uri_t cmd_uri = {
            .uri       = "/api/command",
            .method    = HTTP_GET,
            .handler   = command_api_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &cmd_uri);
        
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

// --- Wi-Fi 初始化修改 ---

// Wi-Fi 事件处理器
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
        // IP获取成功后，启动Web服务器
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

    // 注册事件处理器
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = { };
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init finished.");
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Application startup.");

    // Initialize Wi-Fi
    wifi_init();

    // Initialize I2C bus
    i2cdev_init();

    PCA9685 servo_driver;
    servo_driver.init();

    motion_controller_ptr = std::make_unique<MotionController>(servo_driver);
    motion_controller_ptr->init();

    auto uart_handler_ptr = std::make_unique<UartHandler>(*motion_controller_ptr);
    uart_handler_ptr->init();
    
    ESP_LOGI(TAG, "Initialization complete. All modules started.");

    motion_controller_ptr->queue_command({MOTION_STOP, 0});
    vTaskDelay(pdMS_TO_TICKS(2000));
    // motion_controller_ptr->queue_command({MOTION_FORWARD, 0});
    // 保持主任务运行，以确保 unique_ptr 不会被销毁
    while (true) {
        motion_controller_ptr->servo_test(15, 90); // 15通道总是用来校准舵机，方便机械安装
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}