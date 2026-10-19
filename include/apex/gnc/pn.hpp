#pragma once

#include <cmath>

#include <apex/core/types.hpp>

namespace apex {

// Proportional Navigation: push sideways to the line of sight (LOS), as hard
// as it is swinging, so the swing dies out and the pursuer ends up on a
// collision course.
//
// `r` is the LOS (evader - pursuer), `v_rel` its rate (v_evader - v_pursuer).
//   closing speed  Vc = -(r . v_rel) / |r|       > 0 while the gap shrinks
//   LOS rate       W  = (r x v_rel) / (r . r)    0 when the LOS doesn't swing
//   command        a  = N * Vc * (W x r_hat)
// Returns world-frame acceleration, m/s^2. Not yet limited; see limit_accel().
inline Vec3 pn_command(const Vec3& r, const Vec3& v_rel, double nav_gain) {
    const double rr = r.dot(r);  // |r|^2

    // On top of each other: no LOS direction, and r_hat, W would divide by ~0.
    constexpr double eps = 1e-12;  // m^2
    if (rr < eps) {
        return Vec3::Zero();
    }

    const double range = std::sqrt(rr);
    const Vec3 r_hat = r / range;
    const double closing_speed = -r.dot(v_rel) / range;
    const Vec3 los_rate = r.cross(v_rel) / rr;
    return nav_gain * closing_speed * los_rate.cross(r_hat);
}

// Shrink `a` to length `a_max` if it is longer, keeping its direction
// (scaling, not per-axis clipping, which would bend it).
inline Vec3 limit_accel(const Vec3& a, double a_max) {
    const double length = a.norm();
    if (length <= a_max) {
        return a;
    }
    return a * (a_max / length);
}

}  // namespace apex
