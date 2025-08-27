#include "SrpSoundLocalizer.hpp"
#include "dl_rfft.h"
#include "dsps_wind.h"
#include "esp_heap_caps.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <limits>

// --- Constants
constexpr float PI = 3.14159265358979323846;
constexpr int SOUND_SPEED = 343; // Sound speed in m/s

SrpSoundLocalizer::SrpSoundLocalizer(int sample_rate, int fft_size, float mic_radius)
    : m_sample_rate(sample_rate),
      m_fft_size(fft_size),
      m_mic_radius(mic_radius),
      m_num_mics(4),
      m_num_angles(360),
      m_fft_handle(nullptr),
      m_accumulated_samples(0)
{
    init();
}

SrpSoundLocalizer::~SrpSoundLocalizer() {
    if (m_fft_handle) {
        dl_rfft_f32_deinit((dl_fft_f32_t*)m_fft_handle);
    }
}

void SrpSoundLocalizer::init() {
    m_fft_handle = dl_rfft_f32_init(m_fft_size, MALLOC_CAP_INTERNAL);
    if (!m_fft_handle) return;

    m_window.resize(m_fft_size);
    dsps_wind_hann_f32(m_window.data(), m_fft_size);

    m_internal_buffers.resize(m_num_mics);
    for(auto& buffer : m_internal_buffers) {
        buffer.reserve(m_fft_size * 2); // Reserve space to avoid frequent reallocations
    }

    m_input_float.resize(m_fft_size);
    m_fft_outputs.resize(m_num_mics, std::vector<float>(m_fft_size));
    m_beamformed_sum.resize(m_fft_size);

    m_omega.resize(m_fft_size / 2 + 1);
    for (int k = 0; k < m_fft_size / 2 + 1; ++k) {
        m_omega[k] = 2 * PI * k * m_sample_rate / m_fft_size;
    }

    calculate_tao_table();
}

void SrpSoundLocalizer::calculate_tao_table() {
    m_tao_table.resize(m_num_angles, std::vector<float>(m_num_mics));
    float mic_angles_rad[m_num_mics];
    for(int i = 0; i < m_num_mics; ++i) {
        mic_angles_rad[i] = (i * 90.0) * PI / 180.0;
    }
    for (int angle_deg = 0; angle_deg < m_num_angles; ++angle_deg) {
        float angle_rad = static_cast<float>(angle_deg) * PI / 180.0;
        for (int mic_idx = 0; mic_idx < m_num_mics; ++mic_idx) {
            float cos_angle_diff = cos(angle_rad - mic_angles_rad[mic_idx]);
            m_tao_table[angle_deg][mic_idx] = (m_mic_radius * cos_angle_diff) / SOUND_SPEED;
        }
    }
}

void SrpSoundLocalizer::reset() {
    m_accumulated_samples = 0;
    for (auto& buffer : m_internal_buffers) {
        buffer.clear();
    }
}

bool SrpSoundLocalizer::processChunk(const int16_t* const* mic_chunk_data, size_t chunk_size, int& out_angle, std::function<void(const std::vector<float>&)> result_callback) {
    if (!mic_chunk_data) return false;

    // Append new data to internal buffers
    for (int i = 0; i < m_num_mics; ++i) {
        m_internal_buffers[i].insert(m_internal_buffers[i].end(), mic_chunk_data[i], mic_chunk_data[i] + chunk_size);
    }
    m_accumulated_samples += chunk_size;

    // If we have enough data, perform analysis
    if (m_accumulated_samples >= m_fft_size) {
        out_angle = analyze(result_callback);
        
        // A simple reset. For overlapping windows, you would erase only a portion of the buffer.
        reset();
        return true;
    }

    return false;
}

int SrpSoundLocalizer::analyze(std::function<void(const std::vector<float>&)> result_callback) {
    if (!m_fft_handle) return -1;

    std::vector<float> all_energies(m_num_angles);

    // Step 1: Perform FFT on all microphone channels using data from internal buffers
    for (int mic_idx = 0; mic_idx < m_num_mics; ++mic_idx) {
        for (int i = 0; i < m_fft_size; ++i) {
            m_input_float[i] = static_cast<float>(m_internal_buffers[mic_idx][i]) * m_window[i];
        }
        dl_rfft_f32_run((dl_fft_f32_t*)m_fft_handle, m_input_float.data());
        std::copy(m_input_float.begin(), m_input_float.end(), m_fft_outputs[mic_idx].begin());
    }

    // Step 2: Iterate through all angles and calculate beamformed energy
    for (int angle = 0; angle < m_num_angles; ++angle) {
        std::fill(m_beamformed_sum.begin(), m_beamformed_sum.end(), 0.0f);

        for (int mic_idx = 0; mic_idx < m_num_mics; ++mic_idx) {
            const float* fft_data = m_fft_outputs[mic_idx].data();
            const float tao = m_tao_table[angle][mic_idx];

            float phase_dc = m_omega[0] * tao;
            m_beamformed_sum[0] += fft_data[0] * cosf(phase_dc);
            
            float phase_nyquist = m_omega[m_fft_size / 2] * tao;
            m_beamformed_sum[1] += fft_data[1] * cosf(phase_nyquist);

            for (int k = 1; k < m_fft_size / 2; ++k) {
                float phase = m_omega[k] * tao;
                float h_real = cosf(phase);
                float h_imag = -sinf(phase);
                float fft_real = fft_data[k * 2];
                float fft_imag = fft_data[k * 2 + 1];
                float yk_real = fft_real * h_real - fft_imag * h_imag;
                float yk_imag = fft_real * h_imag + fft_imag * h_real;
                m_beamformed_sum[k * 2] += yk_real;
                m_beamformed_sum[k * 2 + 1] += yk_imag;
            }
        }

        // Step 3: Calculate energy in the frequency domain
        float current_energy = 0.0f;
        current_energy += m_beamformed_sum[0] * m_beamformed_sum[0];
        current_energy += m_beamformed_sum[1] * m_beamformed_sum[1];
        for (int k = 1; k < m_fft_size / 2; ++k) {
            float real = m_beamformed_sum[k * 2];
            float imag = m_beamformed_sum[k * 2 + 1];
            current_energy += (real * real + imag * imag);
        }
        
        all_energies[angle] = current_energy;
    }

    // Find the best angle and max energy for normalization
    float max_energy = 0.0f;
    int best_angle = 0;
    for(int i = 0; i < m_num_angles; ++i) {
        if (all_energies[i] > max_energy) {
            max_energy = all_energies[i];
            best_angle = i;
        }
    }

    // Normalize energies to get probabilities (0.0 to 1.0)
    if (max_energy > 0) {
        for (float& energy : all_energies) {
            energy /= max_energy;
        }
    }

    // Use the callback to pass the results out
    if (result_callback) {
        result_callback(all_energies);
    }

    return best_angle;
}
