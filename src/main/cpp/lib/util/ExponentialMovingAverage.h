#pragma once

class ExponentialMovingAverage {
 public:
  explicit ExponentialMovingAverage(double alpha);

  double Calculate(double value);
  void Reset();

 private:
  double m_alpha;
  double m_average = 0.0;
  bool m_initialized = false;
};
