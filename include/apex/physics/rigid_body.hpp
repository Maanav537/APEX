#pragma once

#include <apex/core/types.hpp>

namespace apex {

// Total force and torque acting on a body at one instant.
struct Wrench {
    Vec3 force = Vec3::Zero();   // world frame, N
    Vec3 torque = Vec3::Zero();  // body frame, N*m
};

// Time derivative of a rigid-body state (6-DOF).
//
// `forces(t, x)` returns the Wrench acting on the body; new force sources
// (gravity, thrust, drag, fins) plug in there without changing this function.
//
//   p-dot = v
//   v-dot = F / m
//   q-dot = 1/2 * q (x) [0, w]
//   w-dot = I^-1 * (tau - w x (I * w))
template <class Forces>
RigidBodyState rigid_body_derivative(double t, const RigidBodyState& x,
                                     const RigidBodyParams& params, Forces&& forces) {
    const Wrench wrench = forces(t, x);
    const Vec3& w = x.angular_velocity;

    const Quat w_pure(0.0, w.x(), w.y(), w.z());
    const Quat q_dot(0.5 * (x.orientation * w_pure).coeffs());

    const Vec3 gyroscopic = w.cross(params.inertia_body * w);
    const Vec3 w_dot = params.inertia_body_inv * (wrench.torque - gyroscopic);

    return {x.velocity, wrench.force / params.mass, q_dot, w_dot};
}

}  // namespace apex
