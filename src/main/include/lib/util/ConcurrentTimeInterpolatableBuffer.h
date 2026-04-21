#pragma once

#include <map>
#include <optional>
#include <functional>
#include <mutex>

template <typename T>
class ConcurrentTimeInterpolatableBuffer {
 public:
  using Interpolator = std::function<T(const T&, const T&, double)>;

  static ConcurrentTimeInterpolatableBuffer Create(Interpolator interp, double historySeconds);
  static ConcurrentTimeInterpolatableBuffer Create(double historySeconds);

  void AddSample(double timestamp, T value);
  void Clear();
  std::optional<T> GetSample(double timestamp) const;
  std::pair<double, T> GetLatest() const;

 private:
  std::map<double, T> m_buffer;
  mutable std::mutex m_mutex;
  Interpolator m_interpolator;
  double m_historySeconds;
};
