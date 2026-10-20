#pragma once

#include <apex/core/types.hpp>

namespace apex {

// Proportional Navigation (pure PN): turn the cat's velocity as fast as the
// line of sight (LOS) is swinging, times N, so the swing dies out and the cat
// ends up on a collision course with the mouse.
//
// `r` is the LOS (mouse - cat), `v_rel` its rate (v_mouse - v_cat),
// `v_cat` the cat's own velocity.
//   LOS rate  W = (r x v_rel) / (r . r)    0 when the LOS doesn't swing
//   command   a = N * (W x v_cat)          sideways to v_cat: turns, never speeds up
// Returns world-frame acceleration, m/s^2. Not yet limited; see limit_accel().
inline Vec3 pn_command(const Vec3& r, const Vec3& v_rel, const Vec3& v_cat, double nav_gain) {
    const double rr = r.dot(r);  // |r|^2

    // On top of each other: no LOS direction, and W would divide by ~0.
    constexpr double eps = 1e-12;  // m^2
    if (rr < eps) {
        return Vec3::Zero();
    }

    const Vec3 los_rate = r.cross(v_rel) / rr;
    return nav_gain * los_rate.cross(v_cat);
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
