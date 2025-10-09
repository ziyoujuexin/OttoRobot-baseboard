#include "WebLogger.hpp"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <vector>
#include <string>
#include <algorithm>
#include <cstdarg>

static const char *TAG = "WebLogger";

// --- Module-level static variables ---

// Original vprintf function pointer
static vprintf_like_t s_original_vprintf = nullptr;

// Queue to buffer log messages from any task to the dispatcher task
static QueueHandle_t s_log_queue = nullptr;

// List of connected WebSocket client file descriptors
static std::vector<int> s_clients;

// Mutex to protect access to the s_clients list
static SemaphoreHandle_t s_clients_mutex = nullptr;

// --- WebSocket and Log Dispatcher Implementation ---

/**
 * @brief The task that forwards logs from the queue to all connected WebSocket clients.
 */
static void log_dispatcher_task(void *pvParameters) {
    httpd_handle_t server = (httpd_handle_t)pvParameters;
    char *log_message = nullptr;
    for (;;) {
        // Wait forever for a new log message to appear in the queue
        if (xQueueReceive(s_log_queue, &log_message, portMAX_DELAY) == pdPASS) {
            if (log_message == nullptr) continue;

            if (xSemaphoreTake(s_clients_mutex, portMAX_DELAY) == pdTRUE) {
                if (!s_clients.empty()) {
                    httpd_ws_frame_t ws_pkt;
                    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
                    ws_pkt.payload = (uint8_t *)log_message;
                    ws_pkt.len = strlen(log_message);
                    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

                    // Asynchronously send to all clients
                    for (int client_fd : s_clients) {
                        httpd_ws_send_frame_async(server, client_fd, &ws_pkt);
                    }
                }
                xSemaphoreGive(s_clients_mutex);
            }
            // Free the message buffer allocated in the vprintf hook
            free(log_message);
        }
    }
}

/**
 * @brief WebSocket handler for the /ws/logs endpoint.
 */
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, new client connected");
        // On Connect: Add client to the list
        if (xSemaphoreTake(s_clients_mutex, portMAX_DELAY) == pdTRUE) {
            s_clients.push_back(httpd_req_to_sockfd(req));
            xSemaphoreGive(s_clients_mutex);
        }
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0); // 0 max_len means payload is not copied
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
        return ret;
    }

    // ESP_OK means a frame was received. Check the type.
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        // On Close: Remove client from the list
        ESP_LOGI(TAG, "Client disconnected");
        int sock_fd = httpd_req_to_sockfd(req);
        if (xSemaphoreTake(s_clients_mutex, portMAX_DELAY) == pdTRUE) {
            s_clients.erase(std::remove(s_clients.begin(), s_clients.end(), sock_fd), s_clients.end());
            xSemaphoreGive(s_clients_mutex);
        }
    }
    // We don't process incoming frames, so we just return OK.
    return ESP_OK;
}

static const httpd_uri_t ws_uri = {
    .uri        = "/ws/logs",
    .method     = HTTP_GET,
    .handler    = ws_handler,
    .user_ctx   = NULL,
    .is_websocket = true
};

/**
 * @brief Custom vprintf-like function that captures logs.
 * 
 * This function does two things:
 * 1. Prints the log to the original serial output.
 * 2. Formats the log and sends it to the s_log_queue.
 */
static int web_log_vprintf(const char *format, va_list args) {
    // Print to the original console
    int ret = s_original_vprintf(format, args);

    // Now, format the message for the web
    va_list args_copy;
    va_copy(args_copy, args);
    int len = vsnprintf(NULL, 0, format, args_copy);
    va_end(args_copy);

    if (len > 0) {
        char *log_buf = (char *)malloc(len + 1);
        if (log_buf) {
            vsnprintf(log_buf, len + 1, format, args);
            // Try to send to the queue, but don't block if it's full
            if (xQueueSend(s_log_queue, &log_buf, (TickType_t)0) != pdPASS) {
                // Queue is full, free the buffer to prevent memory leak
                free(log_buf);
            }
        }
    }
    return ret;
}

// --- Public Class Method ---

void WebLogger::install(httpd_handle_t server) {
    ESP_LOGI(TAG, "Installing WebLogger");

    // Create the mutex for thread-safe client list access
    s_clients_mutex = xSemaphoreCreateMutex();

    // Create a queue to hold 20 log messages
    s_log_queue = xQueueCreate(20, sizeof(char *));

    // Create the task that will dispatch logs from the queue, passing the server handle to it
    xTaskCreate(log_dispatcher_task, "log_dispatcher", 4096, server, 5, NULL);

    // Register the WebSocket handler
    ESP_LOGI(TAG, "Registering WebSocket URI handler: %s", ws_uri.uri);
    httpd_register_uri_handler(server, &ws_uri);

    // Set our custom vprintf function and store the original
    s_original_vprintf = esp_log_set_vprintf(web_log_vprintf);
    
    ESP_LOGI(TAG, "WebLogger installed successfully. Logs will now be sent to /ws/logs");
}
