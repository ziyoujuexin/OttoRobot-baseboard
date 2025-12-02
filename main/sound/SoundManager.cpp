#include "SoundManager.hpp"
#include "esp_log.h"
#include <string>
#include <cstring> // For memcpy
#include "motion_manager/MotionController.hpp"
#include "UartHandler.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "SoundManager";

SoundManager::SoundManager(MotionController* motion_controller, UartHandler* uart_handler)
    : m_is_speaking(false),
      m_last_angle(-1),
      m_processing_task_handle(nullptr),
      m_reaction_task_handle(nullptr),
      m_motion_controller_ptr(motion_controller),
      m_uart_handler_ptr(uart_handler) {

    ESP_LOGI(TAG, "Initializing SoundManager...");

    // Allocate the C-style buffers required by the libraries.
    // These will be deallocated in the destructor.
    m_i2s_buffer.resize(NUM_MICS);
    m_srp_buffer.resize(NUM_MICS);
    for (int i = 0; i < NUM_MICS; ++i) {
        m_i2s_buffer[i] = new int32_t[FRAME_SIZE];
        m_srp_buffer[i] = new int16_t[FRAME_SIZE];
    }
}

SoundManager::~SoundManager() {
    ESP_LOGI(TAG, "Deinitializing SoundManager...");

    if (m_processing_task_handle) {
        vTaskDelete(m_processing_task_handle);
        m_processing_task_handle = nullptr;
    }
    if (m_reaction_task_handle) {
        vTaskDelete(m_reaction_task_handle);
        m_reaction_task_handle = nullptr;
    }

    // Deallocate the buffers
    for (int i = 0; i < NUM_MICS; ++i) {
        delete[] m_i2s_buffer[i];
        delete[] m_srp_buffer[i];
    }
}

void SoundManager::start() {
    // Use std::make_unique for safe, automatic memory management of modules
    m_reader = std::make_unique<DualI2SReader>();
    m_vad = std::make_unique<VAD>(I2S_SAMPLE_RATE, 20); // 20ms frame duration
    m_srp_localizer = std::make_unique<SrpSoundLocalizer>(I2S_SAMPLE_RATE, SRP_FFT_SIZE, MIC_RADIUS);

    m_reader->begin();

    // --- Configure VAD Callback ---
    m_vad->on_vad_state_change([this](bool speaking) {
        ESP_LOGV(TAG, "VAD State Changed: %s", speaking ? "SPEAKING" : "NOT SPEAKING");
        this->m_is_speaking = speaking;
        if (!speaking) {
            // When speech stops, reset the localizer to be ready for the next utterance.
            ESP_LOGV(TAG, "VAD detected speech end, resetting sound localizer.");
            this->m_srp_localizer->reset();
        }
    });

    ESP_LOGI(TAG, "Starting sound processing task.");
    BaseType_t result = xTaskCreatePinnedToCore(
        sound_processing_task_entry,
        "SoundProcTask",
        4096, // Stack size
        this, // Task parameter
        5,    // Priority
        &m_processing_task_handle, 1);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sound processing task");
    }

    ESP_LOGI(TAG, "Starting sound reaction task.");
    result = xTaskCreatePinnedToCore(
        sound_reaction_task_entry,
        "SoundReactTask",
        4096, // Stack size
        this, // Task parameter
        5,    // Priority
        &m_reaction_task_handle, 1);
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sound reaction task");
    }
}

int SoundManager::get_last_detected_angle() const {
    return m_last_angle.load();
}

bool SoundManager::is_idle() const {
    return !m_is_speaking.load();
}

void SoundManager::sound_processing_task_entry(void* arg) {
    static_cast<SoundManager*>(arg)->sound_processing_task();
}

void SoundManager::sound_reaction_task_entry(void* arg) {
    static_cast<SoundManager*>(arg)->sound_reaction_task();
}

void SoundManager::sound_reaction_task() {
    ESP_LOGI(TAG, "Sound reaction task started.");

    while (1) {
        int detected_angle = m_last_angle.load();

        if (detected_angle != -1) {
            // Log the detected angle as requested by the user.
            ESP_LOGI("main", "Sound event processed. Detected Angle: %d", detected_angle);

            // Reset the angle to avoid logging the same angle multiple times.
            m_last_angle.store(-1);
        }

        // Wait for a short period before checking again.
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}

void SoundManager::sound_processing_task() {
    ESP_LOGI(TAG, "Sound processing loop started.");
    while (1) {
        size_t samples_read = m_reader->read(m_i2s_buffer.data(), portMAX_DELAY);
        if (samples_read > 0) {
            // Convert 32-bit I2S data to 16-bit and re-order channels for SRP
            for (size_t i = 0; i < samples_read; i++) {
                m_srp_buffer[0][i] = (int16_t)(m_i2s_buffer[0][i] >> 16);
                m_srp_buffer[1][i] = (int16_t)(m_i2s_buffer[1][i] >> 16);
                m_srp_buffer[2][i] = (int16_t)(m_i2s_buffer[2][i] >> 16);
                m_srp_buffer[3][i] = (int16_t)(m_i2s_buffer[3][i] >> 16);
            }

            // Feed one channel to VAD
            m_vad->feed((const int16_t**)m_srp_buffer.data(), samples_read, 1);

            if (m_is_speaking) {
                ESP_LOGD(TAG, "VAD is active, processing chunk for angle...");
                int angle = -1;
                if (m_srp_localizer->processChunk((const int16_t* const*)m_srp_buffer.data(), samples_read, angle, nullptr)) {
                    ESP_LOGD(TAG, "Sound event processed. Detected Angle: %d", angle);
                    m_last_angle = angle; // Store the detected angle
                    // VAD will set this to false when speech ends. Manually resetting it here can cause issues.
                    // m_is_speaking = false; 
                }
            } else {
                ESP_LOGV(TAG, "VAD not active, skipping angle processing.");
            }
        }
    }
}