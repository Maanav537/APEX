#pragma once

#include <cmath>
#include <limits>

#include <apex/core/types.hpp>
#include <apex/gnc/pn.hpp>

namespace apex {

// Mouse guidance settings. Numbers here are made up for the exercise.
struct EvadeParams {
    double accel_max = 50.0;  // m/s^2, the mouse's own limit
    double t_go_on = 3.0;     // s, weave once the cat is closer than this in time
    double weave_leg = 0.2;   // s, how long each weave direction is held (chosen from a sweep)
};

struct EvadeCommand {
    Vec3 accel = Vec3::Zero();  // m/s^2, world frame, not yet limited (see limit_turn_and_speed)
    bool dodging = false;
    double t_go = std::numeric_limits<double>::infinity();  // s, infinite when not closing
};

// The mouse's rule: push toward the goal, and when the cat is close in time,
// also weave up and down across the LOS so the cat's filter is always behind.
// Summed, not limited: the caller limits it together with the speed push.
//
// `r` is the LOS from the mouse (cat - mouse), `v_rel` its rate (v_cat - v_mouse),
// both world frame. `time_in_weave` is how long the weave has been running
// (0 on its first update).
//   goal push   a_goal = a_max * (g - (g . u_v) u_v)   g = unit to goal, u_v = unit velocity;
//                                                       0 once pointed at the goal
//   time to go  t_go = |r| / Vc,  Vc = -(r . v_rel) / |r|   (closing speed)
//   weave dir   w = world up minus its part along the LOS, unit length
//   weave       a_dodge = +-a_max * w   if t_go < t_go_on; + on even legs (first leg up)
inline EvadeCommand evade_command(const Vec3& r, const Vec3& v_rel, const Vec3& mouse_velocity,
                                  const Vec3& to_goal, double time_in_weave,
                                  const EvadeParams& params = {}) {
    EvadeCommand out;

    const Vec3 u_v = mouse_velocity.normalized();
    const Vec3 g = to_goal.normalized();
    const Vec3 a_goal = params.accel_max * (g - g.dot(u_v) * u_v);

    Vec3 a_dodge = Vec3::Zero();
    const double range = r.norm();
    if (range > 0.0) {
        const Vec3 u_los = r / range;
        const double closing = -r.dot(v_rel) / range;
        if (closing > 0.0) {
            out.t_go = range / closing;
        }
        if (out.t_go < params.t_go_on) {
            // Cat straight above or below: "up across the LOS" vanishes, use world x instead.
            Vec3 w = Vec3::UnitZ() - u_los.z() * u_los;
            if (w.norm() < 1e-6) {
                w = Vec3::UnitX() - u_los.x() * u_los;
            }
            const long long leg = static_cast<long long>(std::floor(time_in_weave / params.weave_leg));
            const double sign = (leg % 2 == 0) ? 1.0 : -1.0;
            a_dodge = sign * params.accel_max * w.normalized();
            out.dodging = true;
        }
    }

    out.accel = a_goal + a_dodge;
    return out;
}

}  // namespace apex
