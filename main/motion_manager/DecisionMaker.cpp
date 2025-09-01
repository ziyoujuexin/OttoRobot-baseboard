#include "DecisionMaker.hpp"
#include "MotionController.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "DecisionMaker";

// Thresholds for face area to trigger forward movement
const int screen_center_x = 640 / 2;
const int screen_center_y = 480 / 2;
const int FORWARD_THRESHOLD_IGNORE = 100*100; // faces small than this will be ignored
const int FORWARD_THRESHOLD_MIN = 200 * 200; // faces small than this area will trigger forward
const int FORWARD_THRESHOLD_MAX = 400 * 680; // faces larger than this area will quit face tracking
const int FORWARD_THRESHOLD_CENTER_X = screen_center_x; // only in the center half of the screen will trigger forward
const int FORWARD_THRESHOLD_CENTER_Y = screen_center_y;

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
        // Check if face tracking should be activated
        if (m_last_face_location.detected && !m_motion_controller.is_face_tracking_active())
        {
            ESP_LOGI(TAG, "Face detected, starting face tracking.");
            m_motion_controller.queue_command({MOTION_FACE_TRACE, {}});
            vTaskDelay(pdMS_TO_TICKS(500)); // Give time for the action to start
            continue; // Re-evaluate immediately
        }

        // Check if the robot is already performing a major body movement
        if (m_motion_controller.is_body_moving())
        {
            // If body is moving, wait a bit before re-evaluating
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // --- Decision Logic ---
        int face_area = m_last_face_location.w * m_last_face_location.h;
        int face_center_x_delta = abs(screen_center_x - (m_last_face_location.x + m_last_face_location.w / 2));
        int face_center_y_delta = abs(screen_center_y - (m_last_face_location.y + m_last_face_location.h / 2));

        if (face_area < FORWARD_THRESHOLD_IGNORE)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (face_area < FORWARD_THRESHOLD_MIN)
        {
            if( face_center_x_delta > FORWARD_THRESHOLD_CENTER_X ||
                face_center_y_delta > FORWARD_THRESHOLD_CENTER_Y )
            {
                vTaskDelay(pdMS_TO_TICKS(200));
                continue; // Face not centered enough, which mean face is in the edge area so box is small
            }
            ESP_LOGI(TAG, "Face too far, moving forward.");
            m_motion_controller.queue_command({MOTION_FORWARD, {}});
            // Give the motion system time to start the action
            vTaskDelay(pdMS_TO_TICKS(1000)); 
        }
        else if(face_area < FORWARD_THRESHOLD_MAX) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue; // Face is close, should quit face tracking
            // this logic wait for host quit
        }
        else { // face_area >= FORWARD_THRESHOLD_MAX
            ESP_LOGI(TAG, "Face close enough, stopping face tracking.");
            m_motion_controller.queue_command({MOTION_STOP, {}});
            // avoid collision of users' face
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}