#include "DecisionMaker.hpp"
#include "MotionController.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "DecisionMaker";

// Thresholds for face area to trigger forward movement
const int FORWARD_THRESHOLD_MIN = 150 * 150;
const int FORWARD_THRESHOLD_MAX = 200 * 200;

DecisionMaker::DecisionMaker(MotionController& motion_controller)
    : m_motion_controller(motion_controller)
    , m_last_face_location({0, 0, 0, 0, false})
{
}

void DecisionMaker::start()
{
    xTaskCreate(
        [](void* arg) {
            static_cast<DecisionMaker*>(arg)->decision_maker_task();
        },
        "decision_maker_task",
        4096,
        this,
        5,
        nullptr);
}

void DecisionMaker::set_face_location(const FaceLocation& location)
{
    // This function can be called from another task (e.g., UartHandler)
    // For now, a simple copy is fine. If complex state is shared, use a mutex.
    m_last_face_location = location;
}

void DecisionMaker::decision_maker_task()
{
    ESP_LOGI(TAG, "Decision maker task started");
    while (true)
    {
        // Check if the robot is already performing a major body movement
        if (m_motion_controller.is_body_moving())
        {
            // If body is moving, wait a bit before re-evaluating
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // --- Decision Logic ---
        int face_area = m_last_face_location.w * m_last_face_location.h;

        if (face_area > FORWARD_THRESHOLD_MIN && face_area < FORWARD_THRESHOLD_MAX)
        {
            ESP_LOGI(TAG, "Face detected at a suitable distance, moving forward.");
            m_motion_controller.queue_command({MOTION_FORWARD, {}});
            // Give the motion system time to start the action
            vTaskDelay(pdMS_TO_TICKS(1000)); 
        }
        else
        {
            // No action needed, wait before checking again
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}