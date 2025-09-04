#pragma once

#include "DualI2SReader.hpp"
#include "VAD.hpp"
#include "SrpSoundLocalizer.hpp"

#include <atomic>
#include <memory>
#include <vector>

#define FRAME_SIZE 320
#define NUM_MICS 4
#define I2S_SAMPLE_RATE 16000
#define SRP_FFT_SIZE 512
#define MIC_RADIUS 0.043f

// Forward declarations
class MotionController;
class UartHandler;

class SoundManager {
public:
    SoundManager(MotionController* motion_controller, UartHandler* uart_handler);
    ~SoundManager();

    /**
     * @brief Starts the background task for sound processing.
     */
    void start();

    /**
     * @brief Gets the last detected sound source angle.
     * @return The angle in degrees (0-359), or -1 if no angle has been detected yet.
     */
    int get_last_detected_angle() const;

private:
    // Task entry points
    static void sound_processing_task_entry(void* arg);
    static void sound_reaction_task_entry(void* arg);
    
    // Main task implementations
    void sound_processing_task();
    void sound_reaction_task();

    // Audio processing modules
    std::unique_ptr<DualI2SReader> m_reader;
    std::unique_ptr<VAD> m_vad;
    std::unique_ptr<SrpSoundLocalizer> m_srp_localizer;

    // Buffers - managed by the class to prevent memory leaks
    std::vector<int32_t*> m_i2s_buffer;
    std::vector<int16_t*> m_srp_buffer;

    // State
    std::atomic<bool> m_is_speaking;
    std::atomic<int> m_last_angle;

    // RTOS
    TaskHandle_t m_processing_task_handle;
    TaskHandle_t m_reaction_task_handle;

    MotionController* m_motion_controller_ptr;
    UartHandler* m_uart_handler_ptr;
};