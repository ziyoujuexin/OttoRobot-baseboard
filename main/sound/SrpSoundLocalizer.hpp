#pragma once

#include <functional>

#include <cstdint>
#include <vector>

/**
 * @class SrpSoundLocalizer
 * @brief
 * 使用频域延迟求和（SRP）算法的声源定位类（有状态，支持流式处理）。
 *
 * 该类内部维护一个缓冲区，可以持续接收小块的音频数据（流式），
 * 当累积的数据量达到FFT点数时，会自动触发一次完整的声源定位分析。
 */
class SrpSoundLocalizer {
public:
    SrpSoundLocalizer(int sample_rate = 16000, int fft_size = 512, float mic_radius = 0.032);
    ~SrpSoundLocalizer();

    /**
     * @brief 处理一个音频数据块（流式输入）。
     *
     * 该方法会累积数据块。当累积的样本数达到 'fft_size' 时，
     * 它会触发一次完整的定位分析，并通过 out_angle 返回结果。
     *
     * @param mic_chunk_data 一个包含4个指针的数组，每个指针指向一路麦克风的数据块。
     * @param chunk_size 当前数据块中的样本数 (例如 VAD 提供的 320)。
     * @param out_angle (输出参数) 如果完成了一次分析，该变量将被更新为计算出的角度。
     * @param result_callback (回调函数) 如果完成分析，则调用此函数并传入包含360个概率值的向量。
     * @return bool 如果完成了一次分析则返回 true，否则返回 false。
     */
    bool processChunk(const int16_t* const* mic_chunk_data, size_t chunk_size, int& out_angle, std::function<void(const std::vector<float>&)> result_callback);

    /**
     * @brief 手动重置内部状态，清空所有已累积的样本。
     */
    void reset();

private:
    void init();
    void calculate_tao_table();
    /**
     * @brief 内部核心分析函数，仅在数据足够时被调用。
     * @param result_callback (回调函数) 将分析结果（概率向量）通过此函数传出。
     * @return int 计算出的角度。
     */
    int analyze(std::function<void(const std::vector<float>&)> result_callback);

    // 配置参数
    const int m_sample_rate;
    const int m_fft_size;
    const float m_mic_radius;
    const int m_num_mics;
    const int m_num_angles;

    // dl_fft 相关
    void* m_fft_handle;

    // 状态和内部缓冲区
    size_t m_accumulated_samples;
    std::vector<std::vector<int16_t>> m_internal_buffers;

    // 计算缓冲区
    std::vector<float> m_window;
    std::vector<float> m_input_float; 
    std::vector<std::vector<float>> m_fft_outputs;
    std::vector<float> m_beamformed_sum;
    
    // 预计算表
    std::vector<std::vector<float>> m_tao_table;
    std::vector<float> m_omega;
};
