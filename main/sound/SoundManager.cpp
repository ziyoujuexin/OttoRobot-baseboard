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
    BaseType_t result = xTaskCreate(
        sound_processing_task_entry,
        "SoundProcTask",
        4096, // Stack size
        this, // Task parameter
        5,    // Priority
        &m_processing_task_handle);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sound processing task");
    }

    ESP_LOGI(TAG, "Starting sound reaction task.");
    result = xTaskCreate(
        sound_reaction_task_entry,
        "SoundReactTask",
        4096, // Stack size
        this, // Task parameter
        5,    // Priority
        &m_reaction_task_handle);
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sound reaction task");
    }
}

int SoundManager::get_last_detected_angle() const {
    return m_last_angle.load();
}

void SoundManager::sound_processing_task_entry(void* arg) {
    static_cast<SoundManager*>(arg)->sound_processing_task();
}

void SoundManager::sound_reaction_task_entry(void* arg) {
    static_cast<SoundManager*>(arg)->sound_reaction_task();
}

void SoundManager::sound_reaction_task() {
    ESP_LOGI(TAG, "Sound reaction task started.");
    int last_processed_angle = -1;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (m_uart_handler_ptr->m_isWakeWordDetected == false) {
            continue; // Skip processing if wake word not detected
        }
        int detected_angle = get_last_detected_angle();

        if (detected_angle != -1) {
            ESP_LOGI(TAG, "New sound detected at angle: %d. Reacting.", detected_angle);

            // --- Head Turn Logic ---
            bool is_left = (detected_angle > 100 && detected_angle <= 270);
            bool is_right = (detected_angle >= 0 && detected_angle < 80) || (detected_angle > 270 && detected_angle < 360);

            if (is_left || is_right) {
                float head_angle = is_left ? 120.0f : 60.0f; // 115 for left, 65 for right
                
                ESP_LOGI(TAG, "Turning head to angle %.1f", head_angle);
                motion_command_t head_turn_cmd;
                head_turn_cmd.motion_type = 0xF0; // MOTION_SERVO_CONTROL
                head_turn_cmd.params.push_back(static_cast<uint8_t>(ServoChannel::HEAD_PAN));
                uint8_t float_bytes[sizeof(float)];
                memcpy(float_bytes, &head_angle, sizeof(float));
                head_turn_cmd.params.insert(head_turn_cmd.params.end(), float_bytes, float_bytes + sizeof(float));
                uint32_t duration_ms = 500; // 500ms to turn head
                uint8_t duration_bytes[sizeof(uint32_t)];
                memcpy(duration_bytes, &duration_ms, sizeof(uint32_t));
                head_turn_cmd.params.insert(head_turn_cmd.params.end(), duration_bytes, duration_bytes + sizeof(uint32_t));
                m_motion_controller_ptr->queue_command(head_turn_cmd);
                vTaskDelay(pdMS_TO_TICKS(500)); // Wait 300ms for head to turn
            }
            
            // --- Original Body Turn Logic ---
            if (detected_angle >= 0 && detected_angle < 80) {
                // right forward
                ESP_LOGI(TAG, "Detected angle: %d, turning right.", detected_angle);
                m_motion_controller_ptr->queue_command({MOTION_TRACKING_R, {}});
                m_uart_handler_ptr->m_isWakeWordDetected = false;
                vTaskDelay(pdMS_TO_TICKS(20)); // At least delay 20ms for msg transmission
            } else if (detected_angle > 100 && detected_angle <= 180) {
                // left forward
                ESP_LOGI(TAG, "Detected angle: %d, turning left.", detected_angle);
                m_motion_controller_ptr->queue_command({MOTION_TRACKING_L, {}});
                m_uart_handler_ptr->m_isWakeWordDetected = false;
                vTaskDelay(pdMS_TO_TICKS(20));
            } else if (detected_angle > 270 && detected_angle < 360) {
                // right backward, leave forward deadzone but needn't for backward
                ESP_LOGI(TAG, "Detected angle: %d, turning right.", detected_angle);
                m_motion_controller_ptr->queue_command({MOTION_TRACKING_R, {}});
                m_motion_controller_ptr->queue_command({MOTION_TRACKING_R, {}});
                m_uart_handler_ptr->m_isWakeWordDetected = false;
                vTaskDelay(pdMS_TO_TICKS(40)); // Double wait time
            } else if (detected_angle > 180 && detected_angle <= 270) {
                // left backward
                ESP_LOGI(TAG, "Detected angle: %d, turning left.", detected_angle);
                m_motion_controller_ptr->queue_command({MOTION_TRACKING_L, {}});
                m_motion_controller_ptr->queue_command({MOTION_TRACKING_L, {}});
                m_uart_handler_ptr->m_isWakeWordDetected = false;
                vTaskDelay(pdMS_TO_TICKS(40));
            }

            last_processed_angle = detected_angle;
            m_last_angle = -1; // Reset the angle after processing
            
            while (!m_motion_controller_ptr->is_idle()) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            ESP_LOGI(TAG, "Sound Reaction complete, stopping.");
            m_motion_controller_ptr->queue_command({MOTION_STOP, {}});
        }
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