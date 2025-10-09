#include "web_server/WebServer.hpp"
#include "web_server/WebLogger.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "config.h"
#include "esp_http_server.h"
#include "json_parser.h"
#include <string>
#include <vector>
#include <dirent.h>
#include "freertos/task.h" // For vTaskDelay
#include "../display/AnimationPlayer.h" // For AnimationPlayer class
#include "../UIManager.hpp" // For UiCommand struct definition

static const char *TAG = "WebServer";

// Forward declarations for static handlers
static esp_err_t tuning_api_handler(httpd_req_t *req);
static esp_err_t command_api_handler(httpd_req_t *req);
static esp_err_t servo_api_handler(httpd_req_t *req);
static esp_err_t root_handler(httpd_req_t *req);
static esp_err_t upload_handler(httpd_req_t *req);
static esp_err_t animations_api_handler(httpd_req_t *req);
static esp_err_t play_animation_handler(httpd_req_t *req);
static esp_err_t delete_animation_handler(httpd_req_t *req);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

class WebServerImpl {
public:
    WebServerImpl(ActionManager& action_manager, MotionController& motion_controller, AnimationPlayer& animation_player) 
        : m_action_manager(action_manager), m_motion_controller(motion_controller), m_animation_player(animation_player) {}

    void start() {
        wifi_init();
    }

private:
    ActionManager& m_action_manager;
    MotionController& m_motion_controller;
    AnimationPlayer& m_animation_player;
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
        config.max_uri_handlers = 10; // Increased from default 8 to accommodate the new WS handler

        ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
        if (httpd_start(&m_server, &config) == ESP_OK) {
            httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = this };
            httpd_register_uri_handler(m_server, &root);

            httpd_uri_t control = { .uri = "/control", .method = HTTP_POST, .handler = command_api_handler, .user_ctx = this };
            httpd_register_uri_handler(m_server, &control);

            httpd_uri_t tune = { .uri = "/api/tune", .method = HTTP_POST, .handler = tuning_api_handler, .user_ctx = this };
            httpd_register_uri_handler(m_server, &tune);

            httpd_uri_t servo_uri = { .uri = "/servo", .method = HTTP_POST, .handler = servo_api_handler, .user_ctx = this };
            httpd_register_uri_handler(m_server, &servo_uri);

            httpd_uri_t upload_uri = { .uri = "/upload", .method = HTTP_POST, .handler = upload_handler, .user_ctx = this };
            httpd_register_uri_handler(m_server, &upload_uri);

            httpd_uri_t animations_uri = { .uri = "/api/animations", .method = HTTP_GET, .handler = animations_api_handler, .user_ctx = this };
            httpd_register_uri_handler(m_server, &animations_uri);

            httpd_uri_t play_animation_uri = { .uri = "/api/play", .method = HTTP_POST, .handler = play_animation_handler, .user_ctx = this };
            httpd_register_uri_handler(m_server, &play_animation_uri);

            httpd_uri_t delete_animation_uri = { .uri = "/api/delete", .method = HTTP_GET, .handler = delete_animation_handler, .user_ctx = this };
            httpd_register_uri_handler(m_server, &delete_animation_uri);

            // Install the web logger to capture and forward logs
            WebLogger::install(m_server);

            return;
        }
        ESP_LOGI(TAG, "Error starting server!");
    }

    friend esp_err_t servo_api_handler(httpd_req_t *req);

    friend esp_err_t tuning_api_handler(httpd_req_t *req);
    friend esp_err_t command_api_handler(httpd_req_t *req);
    friend esp_err_t root_handler(httpd_req_t *req);
    friend esp_err_t play_animation_handler(httpd_req_t *req);
    friend esp_err_t delete_animation_handler(httpd_req_t *req);
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

esp_err_t animations_api_handler(httpd_req_t *req) {
    std::string json_response = "[";
    bool first = true;

    DIR* dir = opendir("/sdcard/animations");
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open animations directory");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open animations directory");
        return ESP_FAIL;
    }

    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL) {
        std::string filename(ent->d_name);
        if (filename.length() > 4 && filename.substr(filename.length() - 4) == ".gif") {
            if (!first) {
                json_response += ",";
            }
            json_response += "\"";
            json_response += filename;
            json_response += "\"";
            first = false;
        }
    }
    closedir(dir);

    json_response += "]";
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response.c_str(), json_response.length());
    return ESP_OK;
}

esp_err_t play_animation_handler(httpd_req_t *req) {
    WebServerImpl* server = (WebServerImpl*)req->user_ctx;
    char content[128];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to receive data");
        return ESP_FAIL;
    }
    content[ret] = '\0';

    jparse_ctx_t jctx;
    json_parse_start(&jctx, content, ret);

    char anim_name_buf[64];
    if (json_obj_get_string(&jctx, "animation", anim_name_buf, sizeof(anim_name_buf)) == 0) {
        ESP_LOGI(TAG, "Requesting one-shot animation '%s' from web.", anim_name_buf);
        server->m_animation_player.playOneShotAnimation(anim_name_buf);
        httpd_resp_send(req, "Animation request sent to player", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON: missing 'animation' key");
    return ESP_FAIL;
}

esp_err_t delete_animation_handler(httpd_req_t *req) {
    char file_buf[128];
    if (httpd_req_get_url_query_str(req, file_buf, sizeof(file_buf)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing file query parameter");
        return ESP_FAIL;
    }

    char param_val[100];
    if (httpd_query_key_value(file_buf, "file", param_val, sizeof(param_val)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to parse file parameter");
        return ESP_FAIL;
    }

    // Security check: ensure no path traversal
    std::string filename(param_val);
    if (filename.find("/") != std::string::npos || filename.find("..") != std::string::npos) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }

    std::string full_path = "/sdcard/animations/" + filename;
    ESP_LOGI(TAG, "Attempting to delete file: %s", full_path.c_str());

    if (remove(full_path.c_str()) == 0) {
        ESP_LOGI(TAG, "Successfully deleted file: %s", full_path.c_str());
        httpd_resp_send(req, "File deleted successfully", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to delete file: %s", full_path.c_str());
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to delete file");
        return ESP_FAIL;
    }
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
        motion_command_t cmd = { .motion_type = (uint8_t)atoi(param_val) };
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

esp_err_t servo_api_handler(httpd_req_t *req)
{
    WebServerImpl* server = (WebServerImpl*)req->user_ctx;
    char content[100];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[ret] = '\0';

    jparse_ctx_t jctx;
    json_parse_start(&jctx, content, ret);

    int channel, angle;
    if (json_obj_get_int(&jctx, "channel", &channel) == 0 &&
        json_obj_get_int(&jctx, "angle", &angle) == 0) {
        
        if (channel >= 0 && channel < 16 && angle >= 0 && angle <= 180) {
            server->m_motion_controller.set_single_servo(static_cast<uint8_t>(channel), static_cast<uint8_t>(angle));
            httpd_resp_send(req, "Servo command OK", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON or parameters");
    return ESP_FAIL;
}

// Helper to find a substring in a buffer
static const char* memmem(const char* haystack, size_t haystack_len, const char* needle, size_t needle_len) {
    if (needle_len == 0) return haystack;
    if (haystack_len < needle_len) return NULL;
    const char* h_end = haystack + haystack_len - needle_len + 1;
    for (const char* h = haystack; h < h_end; ++h) {
        if (memcmp(h, needle, needle_len) == 0) {
            return h;
        }
    }
    return NULL;
}

esp_err_t upload_handler(httpd_req_t *req) {
    char buf[1024];
    char filepath[256];
    FILE* f = NULL;
    int received;

    // Get the boundary
    char boundary[70];
    if (httpd_req_get_hdr_value_str(req, "Content-Type", buf, sizeof(buf)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content-Type header missing");
        return ESP_FAIL;
    }
    char* boundary_start = strstr(buf, "boundary=");
    if (!boundary_start) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Boundary not found in Content-Type");
        return ESP_FAIL;
    }
    boundary_start += strlen("boundary=");
    snprintf(boundary, sizeof(boundary), "--%s", boundary_start);

    size_t boundary_len = strlen(boundary);
    bool is_first_chunk = true;
    bool headers_parsed = false;
    bool is_gif = false;

    ESP_LOGI(TAG, "Starting file upload process. Content length: %d", req->content_len);

    while ((received = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        // Add a small delay to yield to other tasks and ease pressure on the SDIO driver
        vTaskDelay(pdMS_TO_TICKS(5));
        char* ptr = buf;
        int remaining = received;

        if (is_first_chunk) {
            // Skip the first boundary
            const char* boundary_end = memmem(ptr, remaining, boundary, boundary_len);
            if (boundary_end) {
                ptr = (char*)boundary_end + boundary_len;
                remaining -= (ptr - buf);
            }
            is_first_chunk = false;
        }

        if (!headers_parsed) {
            const char* header_end = memmem(ptr, remaining, "\r\n\r\n", 4);
            if (header_end) {
                // Extract filename
                const char* filename_start = memmem(ptr, header_end - ptr, "filename=\"", 10);
                if (filename_start) {
                    filename_start += 10;
                    const char* filename_end = memmem(filename_start, header_end - filename_start, "\"", 1);
                    if (filename_end) {
                        size_t len = filename_end - filename_start;
                        snprintf(filepath, sizeof(filepath), "/sdcard/animations/%.*s", len, filename_start);
                        ESP_LOGI(TAG, "File will be saved to: %s", filepath);
                    } else {
                        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Malformed filename in Content-Disposition");
                        return ESP_FAIL;
                    }
                } else {
                    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Filename not found in Content-Disposition");
                    return ESP_FAIL;
                }

                // Check GIF magic bytes
                const char* data_start = header_end + 4;
                if ((data_start - ptr + 6) <= remaining) {
                    if (memcmp(data_start, "GIF87a", 6) == 0 || memcmp(data_start, "GIF89a", 6) == 0) {
                        is_gif = true;
                        ESP_LOGI(TAG, "GIF magic bytes validated.");
                    } else {
                        ESP_LOGE(TAG, "Not a valid GIF file.");
                        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "File is not a valid GIF");
                        return ESP_FAIL;
                    }
                }

                f = fopen(filepath, "wb");
                if (!f) {
                    ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open file on server");
                    return ESP_FAIL;
                }
                
                size_t data_to_write = remaining - (data_start - ptr);
                fwrite(data_start, 1, data_to_write, f);
                
                ptr = (char*)data_start + data_to_write;
                remaining = 0;
                headers_parsed = true;
            }
        }
        
        if (headers_parsed && f) {
            // Check for the final boundary in the current buffer
            const char* end_boundary = memmem(ptr, remaining, boundary, boundary_len);
            if (end_boundary) {
                size_t data_to_write = end_boundary - ptr;
                fwrite(ptr, 1, data_to_write, f);
                ESP_LOGI(TAG, "Final boundary found. Upload finished.");
                break; // End of file
            } else {
                fwrite(ptr, 1, remaining, f);
            }
        }
    }

    if (f) {
        fclose(f);
    }

    if (received < 0) {
        ESP_LOGE(TAG, "File reception failed!");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File reception failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "File upload successful");
    httpd_resp_send(req, "Upload successful", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
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

WebServer::WebServer(ActionManager& action_manager, MotionController& motion_controller, AnimationPlayer& animation_player)
    : m_action_manager(action_manager), m_motion_controller(motion_controller), m_animation_player(animation_player) {}

void WebServer::start() {
    WebServerImpl* impl = new WebServerImpl(m_action_manager, m_motion_controller, m_animation_player);
    impl->start();
}