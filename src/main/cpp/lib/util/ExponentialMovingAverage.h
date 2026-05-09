#pragma once

class ExponentialMovingAverage {
public:
    explicit ExponentialMovingAverage(double alpha) : m_alpha(alpha) {}

    double Calculate(double value) {
        if (!m_initialized) {
            m_average = value;
            m_initialized = true;
            return value;
        }

        m_average += m_alpha * (value - m_average);
        return m_average;
    }

    void Reset() {
        m_average = 0.0;
        m_initialized = false;
    }

private:
    double m_alpha;
    double m_average = 0.0;
    bool m_initialized = false;
};
