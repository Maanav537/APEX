#pragma once

#include <algorithm>
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

// The cat's guess that the mouse is weaving, from its observer's a_hat alone:
// the part of a_hat across the LOS is over `threshold`. The guess is held for
// `hold` seconds after it was last true, so it doesn't drop out between legs.
struct WeaveDetector {
    double threshold = 20.0;  // m/s^2; the mouse's weave pushes 50
    double hold = 0.5;        // s

    // `r` is the LOS (mouse - cat). Returns whether the cat thinks a weave is on at time `t`.
    bool update(double t, const Vec3& a_hat, const Vec3& r) {
        const double range = r.norm();
        if (range > 0.0) {
            const Vec3 u_los = r / range;
            if ((a_hat - a_hat.dot(u_los) * u_los).norm() > threshold) {
                last_seen_ = t;
            }
        }
        return t - last_seen_ <= hold;
    }

private:
    double last_seen_ = -std::numeric_limits<double>::infinity();
};

}  // namespace apex
