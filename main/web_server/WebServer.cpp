#include "web_server/WebServer.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "config.h"
#include "esp_http_server.h"
#include "json_parser.h"
#include <string>

static const char *TAG = "WebServer";

// Forward declarations for static handlers
static esp_err_t tuning_api_handler(httpd_req_t *req);
static esp_err_t command_api_handler(httpd_req_t *req);
static esp_err_t root_handler(httpd_req_t *req);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

class WebServerImpl {
public:
    WebServerImpl(ActionManager& action_manager, MotionController& motion_controller) 
        : m_action_manager(action_manager), m_motion_controller(motion_controller) {}

    void start() {
        wifi_init();
    }

private:
    ActionManager& m_action_manager;
    MotionController& m_motion_controller;
    httpd_handle_t m_server = NULL;

    void wifi_init() {
        // This should be called only once in the app. 
        // Assuming this is the only component that needs it.
        // A better approach would be to have a central init function in main.
        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        esp_event_handler_instance_t instance_any_id, instance_got_ip;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, this, &instance_any_id));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, this, &instance_got_ip));

        wifi_config_t wifi_config = { };
        strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
        strcpy((char*)wifi_config.sta.password, WIFI_PASSWORD);
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "wifi_init finished.");
    }

    void start_webserver() {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.lru_purge_enable = true;

        ESP_LOGI(TAG, "Starting server on port: \'%d\'", config.server_port);
        if (httpd_start(&m_server, &config) == ESP_OK) {
            httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = this };
            httpd_register_uri_handler(m_server, &root);

            httpd_uri_t control = { .uri = "/control", .method = HTTP_POST, .handler = command_api_handler, .user_ctx = this };
            httpd_register_uri_handler(m_server, &control);

            httpd_uri_t tune = { .uri = "/api/tune", .method = HTTP_POST, .handler = tuning_api_handler, .user_ctx = this };
            httpd_register_uri_handler(m_server, &tune);
            return;
        }
        ESP_LOGI(TAG, "Error starting server!");
    }

    friend esp_err_t tuning_api_handler(httpd_req_t *req);
    friend esp_err_t command_api_handler(httpd_req_t *req);
    friend esp_err_t root_handler(httpd_req_t *req);
    friend void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
};

// --- Static Handlers --- 

esp_err_t root_handler(httpd_req_t *req)
{
    const uint32_t index_html_len = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html_start, index_html_len);
    return ESP_OK;
}

esp_err_t command_api_handler(httpd_req_t *req)
{
    WebServerImpl* server = (WebServerImpl*)req->user_ctx;
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    char param_val[32];
    if (httpd_query_key_value(buf, "motion", param_val, sizeof(param_val)) == ESP_OK) {
        motion_command_t cmd = { .motion_type = (uint8_t)atoi(param_val), .param = 0 };
        server->m_motion_controller.queue_command(cmd);
        httpd_resp_send(req, "Command Queued", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

esp_err_t tuning_api_handler(httpd_req_t *req)
{
    WebServerImpl* server = (WebServerImpl*)req->user_ctx;
    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) { if (ret == HTTPD_SOCK_ERR_TIMEOUT) httpd_resp_send_408(req); return ESP_FAIL; }
    content[ret] = '\0';

    jparse_ctx_t jctx;
    json_parse_start(&jctx, content, ret);

    char cmd_buf[32];
    if (json_obj_get_string(&jctx, "command", cmd_buf, sizeof(cmd_buf)) != 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'command'");
        return ESP_FAIL;
    }

    std::string command(cmd_buf);
    if (command == "get_params") {
        char action_buf[64];
        if (json_obj_get_string(&jctx, "action", action_buf, sizeof(action_buf)) == 0) {
            std::string json_response = server->m_action_manager.get_action_params_json(action_buf);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, json_response.c_str(), json_response.length());
            return ESP_OK;
        }
    } else if (command == "tune_param") {
        char action_buf[64], param_type_buf[32];
        int servo_index; float value;
        if (json_obj_get_string(&jctx, "action", action_buf, sizeof(action_buf)) == 0 &&
            json_obj_get_int(&jctx, "servo", &servo_index) == 0 &&
            json_obj_get_string(&jctx, "param", param_type_buf, sizeof(param_type_buf)) == 0 &&
            json_obj_get_float(&jctx, "value", &value) == 0) {
            server->m_action_manager.tune_gait_parameter(action_buf, servo_index, param_type_buf, value);
            httpd_resp_send(req, "Tune OK", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    } else if (command == "update_action_props") {
        char action_buf[64];
        bool is_atomic;
        int default_steps, gait_period_ms;
        if (json_obj_get_string(&jctx, "action", action_buf, sizeof(action_buf)) == 0 &&
            json_obj_get_bool(&jctx, "is_atomic", &is_atomic) == 0 &&
            json_obj_get_int(&jctx, "default_steps", &default_steps) == 0 &&
            json_obj_get_int(&jctx, "gait_period_ms", &gait_period_ms) == 0) {
            server->m_action_manager.update_action_properties(action_buf, is_atomic, default_steps, gait_period_ms);
            httpd_resp_send(req, "Update OK", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    } else if (command == "save_params") {
        char action_buf[64];
        if (json_obj_get_string(&jctx, "action", action_buf, sizeof(action_buf)) == 0) {
            bool success = server->m_action_manager.save_action_to_nvs(action_buf);
            httpd_resp_set_type(req, "application/json");
            if (success) httpd_resp_send(req, "{\"success\":true}", HTTPD_RESP_USE_STRLEN);
            else httpd_resp_send(req, "{\"success\":false, \"error\":\"Failed to save action to NVS\"}", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return ESP_FAIL;
}

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    WebServerImpl* server = (WebServerImpl*)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Retrying to connect to the AP");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        server->start_webserver();
    }
}

// --- Public WebServer Class --- 

WebServer::WebServer(ActionManager& action_manager, MotionController& motion_controller)
    : m_action_manager(action_manager), m_motion_controller(motion_controller) {}

void WebServer::start() {
    WebServerImpl* impl = new WebServerImpl(m_action_manager, m_motion_controller);
    impl->start();
}