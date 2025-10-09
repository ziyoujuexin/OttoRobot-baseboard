#pragma once

#include "esp_http_server.h"

/**
 * @brief Handles capturing ESP_LOG output and forwarding it over a WebSocket.
 */
class WebLogger {
public:
    /**
     * @brief Installs the web logger.
     * 
     * This function sets up a vprintf hook to capture log output, creates a task
     * to dispatch logs, and registers a WebSocket URI handler with the specified
     * HTTP server.
     * 
     * @param server The httpd_handle_t of the running web server.
     */
    static void install(httpd_handle_t server);
};
