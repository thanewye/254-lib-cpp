#pragma once

#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <utility>

#include <units/time.h>
#include <wpi/MathExtras.h>

/**
 * Thread-safe map of timestamped values that interpolates between samples.
 * Samples older than historySize are discarded on each write.
 *
 * @tparam T Value type. Must support wpi::Lerp unless a custom interpolator is provided.
 */
template<typename T> class ConcurrentTimeInterpolatableBuffer {
public:
    using Interpolator = std::function<T(const T&, const T&, double)>;

    /** Creates a buffer with a custom interpolation function. */
    [[nodiscard]] static ConcurrentTimeInterpolatableBuffer Create(Interpolator interp, units::second_t historySize) {
        return ConcurrentTimeInterpolatableBuffer(std::move(interp), historySize);
    }

    /** Creates a buffer using wpi::Lerp as the interpolation function. */
    [[nodiscard]] static ConcurrentTimeInterpolatableBuffer Create(units::second_t historySize) {
        return ConcurrentTimeInterpolatableBuffer([](const T& a, const T& b, double t) { return wpi::Lerp(a, b, t); }, historySize);
    }

    /** Adds a sample and evicts any entries older than historySize. */
    void AddSample(double timestamp, T value) {
        std::lock_guard lock(mutex);
        buffer[timestamp] = std::move(value);
        CleanUp(timestamp);
    }

    /** Removes all samples. */
    void Clear() {
        std::lock_guard lock(mutex);
        buffer.clear();
    }

    /**
     * Returns the interpolated value at the given timestamp, or nullopt if the
     * buffer is empty. Clamps to the nearest edge sample if timestamp is outside
     * the recorded range.
     */
    [[nodiscard]] std::optional<T> GetSample(double timestamp) const {
        std::lock_guard lock(mutex);
        if (buffer.empty()) {
            return std::nullopt;
        }

        auto it = buffer.find(timestamp);
        if (it != buffer.end()) {
            return it->second;
        }

        auto topIt = buffer.upper_bound(timestamp);

        if (topIt == buffer.end() && topIt == buffer.begin()) {
            return std::nullopt;
        }
        if (topIt == buffer.end()) {
            return std::prev(topIt)->second;
        }
        if (topIt == buffer.begin()) {
            return topIt->second;
        }
        auto bottomIt = std::prev(topIt);
        double t = (timestamp - bottomIt->first) / (topIt->first - bottomIt->first);
        return interpolator(bottomIt->second, topIt->second, t);
    }

    /** Returns the most recent {timestamp, value} pair, or nullopt if empty. */
    [[nodiscard]] std::optional<std::pair<double, T>> GetLatest() const {
        std::lock_guard lock(mutex);
        if (buffer.empty()) {
            return std::nullopt;
        }
        auto it = buffer.rbegin();
        return std::make_pair(it->first, it->second);
    }

    /** Returns a snapshot copy of the internal buffer. Used for replaying odometry inputs. */
    [[nodiscard]] std::map<double, T> GetInternalBuffer() const {
        std::lock_guard lock(mutex);
        return buffer;
    }

private:
    ConcurrentTimeInterpolatableBuffer(Interpolator interp, units::second_t historySize)
        : interpolator(std::move(interp))
        , historySize(historySize) {}

    void CleanUp(double time) {
        double cutoff = time - historySize.value();
        while (!buffer.empty() && buffer.begin()->first < cutoff) {
            buffer.erase(buffer.begin());
        }
    }

    std::map<double, T> buffer;
    mutable std::mutex mutex;
    Interpolator interpolator;
    units::second_t historySize;
};
