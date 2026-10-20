#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <apex/core/types.hpp>

namespace apex {

// Speed limits for one side, as fractions of its cruise speed so any cruise
// speed works. Numbers here are made up for the exercise.
// Defaults are the cat's: 50 m/s cruise -> 40 juke, never below 30.
struct SpeedParams {
    double juke_fraction = 0.8;  // slow to this x cruise while juking
    double min_fraction = 0.6;   // never slower than this x cruise
    double accel_limit = 10.0;   // m/s^2, brake and throttle, also the reserve in the cap
};

// Mouse: 20 m/s cruise -> 15 juke, never below 10, 5 m/s^2.
inline constexpr SpeedParams mouse_speed_params{0.75, 0.5, 5.0};

// Speed push toward `target`: enough to get there in one guidance period
// `dt`, limited to +-accel_limit, so it never overshoots.
//   a_speed = clamp((v_target - v) / dt, -limit, +limit)
inline double speed_command(double speed, double target, double dt, double accel_limit) {
    return std::clamp((target - speed) / dt, -accel_limit, accel_limit);
}

// The cat's guess that the mouse is weaving, from its observer's innovation
// e = z - p_pred (how far each reading lands from where the filter predicted).
//
// A weave the filter's model doesn't expect pushes e the same way for most of a
// leg, then flips; noise flips sign at random, about every other update. So:
//   1. split e into its two parts across the LOS: side = LOS x world-up,
//      up = side x LOS (world-x stands in for world-up when the LOS is nearly vertical)
//   2. per axis, count sign flips over the last `window` updates (0.2 s at 100 Hz)
//   3. a window is quiet if either axis has <= max_flips flips
//   4. `confirm` quiet windows in a row -> weave seen
// The guess is held for `hold` seconds after it was last seen, so it doesn't
// drop out between legs. Fixed-size buffers: update() allocates nothing.
struct WeaveDetector {
    static constexpr int window = 20;  // updates (flip gaps) per window; 0.2 s at 100 Hz
    int max_flips = 3;
    int confirm = 3;     // quiet windows in a row
    double hold = 0.5;   // s

    // `e` is the observer's innovation, `r` the LOS (mouse - cat), both world frame.
    // Call once per observer update. Returns whether the cat thinks a weave is on at time `t`.
    bool update(double t, const Vec3& e, const Vec3& r) {
        const double range = r.norm();
        if (range > 0.0) {
            const Vec3 u = r / range;
            const Vec3 ref = std::abs(u.z()) > 0.99 ? Vec3::UnitX() : Vec3::UnitZ();
            const Vec3 side = u.cross(ref).normalized();
            const Vec3 up = side.cross(u);

            signs_[0][head_] = e.dot(up) >= 0.0;
            signs_[1][head_] = e.dot(side) >= 0.0;
            head_ = (head_ + 1) % (window + 1);
            count_ = std::min(count_ + 1, window + 1);

            bool quiet = false;
            if (count_ == window + 1) {
                for (const auto& axis : signs_) {
                    int flips = 0;
                    // Oldest to newest: head_ is the oldest slot once the ring is full.
                    for (int k = 1; k <= window; ++k) {
                        flips += axis[(head_ + k) % (window + 1)] !=
                                 axis[(head_ + k - 1) % (window + 1)];
                    }
                    quiet = quiet || flips <= max_flips;
                }
            }
            quiet_run_ = quiet ? quiet_run_ + 1 : 0;
            if (quiet_run_ >= confirm) {
                last_seen_ = t;
            }
        }
        return t - last_seen_ <= hold;
    }

private:
    // Ring buffers of signs (true = +): window + 1 samples give `window` flip gaps.
    std::array<std::array<bool, window + 1>, 2> signs_{};
    int head_ = 0;   // next slot to write = oldest sample once full
    int count_ = 0;
    int quiet_run_ = 0;
    double last_seen_ = -std::numeric_limits<double>::infinity();
};

}  // namespace apex
