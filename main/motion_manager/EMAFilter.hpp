#pragma once

#include <cmath>

class EMAFilter {
public:
    EMAFilter(float alpha = 0.5f, float initial_value = 0.0f)
        : m_alpha(alpha), m_filtered_value(initial_value) {}

    float apply(float input_value) {
        if (std::isnan(m_filtered_value)) {
            m_filtered_value = input_value;
        } else {
            m_filtered_value = m_alpha * input_value + (1.0f - m_alpha) * m_filtered_value;
        }
        return m_filtered_value;
    }

    void set_alpha(float alpha) {
        m_alpha = alpha;
    }

    float get_alpha() const {
        return m_alpha;
    }

    void reset(float initial_value = 0.0f) {
        m_filtered_value = initial_value;
    }

private:
    float m_alpha;
    float m_filtered_value;
};
