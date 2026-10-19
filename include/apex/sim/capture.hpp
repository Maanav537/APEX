#pragma once

#include <algorithm>

#include <apex/core/types.hpp>

namespace apex {

// Cat catches mouse when they come this close during a tick.
inline constexpr double capture_radius = 0.5;  // m

// Mouse wins when it comes this close to its goal during a tick.
inline constexpr double goal_radius = 0.5;  // m

// Smallest cat-mouse distance during one tick, not just at its ends.
//
// `r0`, `r1` are the gap (mouse - cat) at the start and end of the tick. The gap
// is taken to move in a straight line, r(s) = r0 + s * d with d = r1 - r0 and
// s in [0, 1]. Minimizing |r(s)|^2 gives s* = -(r0 . d) / (d . d), clamped to
// the tick: s* < 0 means closest was already behind us, s* > 1 means still ahead.
inline double closest_distance(const Vec3& r0, const Vec3& r1) {
    const Vec3 d = r1 - r0;
    const double dd = d.dot(d);  // |d|^2

    // Gap didn't change this tick: every s gives |r0|, and s* would divide by ~0.
    constexpr double eps = 1e-12;  // m^2
    if (dd < eps) {
        return r0.norm();
    }

    const double s = std::clamp(-r0.dot(d) / dd, 0.0, 1.0);
    return (r0 + s * d).norm();
}

}  // namespace apex
