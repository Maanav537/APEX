#pragma once

#include <Eigen/Dense>

namespace apex {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

// State of a point-mass at one instant. The same type also holds its time
// derivative (velocity, acceleration), so integrators can form `x + dt * k`.
struct PointMassState {
    Vec3 position = Vec3::Zero();
    Vec3 velocity = Vec3::Zero();
};

inline PointMassState operator+(const PointMassState& a, const PointMassState& b) {
    return {a.position + b.position, a.velocity + b.velocity};
}

inline PointMassState operator*(double s, const PointMassState& x) {
    return {s * x.position, s * x.velocity};
}

inline PointMassState operator*(const PointMassState& x, double s) {
    return s * x;
}

using Quat = Eigen::Quaterniond;

// State of a rigid body at one instant (6-DOF). Like PointMassState, the same
// type holds its derivative; then `orientation` holds q-dot, which is not unit
// length. Normalizing q after each step is the simulation's job, not RK4's.
struct RigidBodyState {
    Vec3 position = Vec3::Zero();          // world frame
    Vec3 velocity = Vec3::Zero();          // world frame
    Quat orientation = Quat::Identity();   // body -> world rotation
    Vec3 angular_velocity = Vec3::Zero();  // body frame
};

// Eigen quaternions have no `+` or scalar `*`, so these act on the four
// coefficients directly.
inline RigidBodyState operator+(const RigidBodyState& a, const RigidBodyState& b) {
    return {a.position + b.position,
            a.velocity + b.velocity,
            Quat(a.orientation.coeffs() + b.orientation.coeffs()),
            a.angular_velocity + b.angular_velocity};
}

inline RigidBodyState operator*(double s, const RigidBodyState& x) {
    return {s * x.position,
            s * x.velocity,
            Quat(s * x.orientation.coeffs()),
            s * x.angular_velocity};
}

inline RigidBodyState operator*(const RigidBodyState& x, double s) {
    return s * x;
}

// Fixed properties of a rigid body; not integrated. Build with
// make_rigid_body_params() so the inverse inertia always matches.
struct RigidBodyParams {
    double mass = 1.0;                         // kg
    Mat3 inertia_body = Mat3::Identity();      // kg*m^2, body frame
    Mat3 inertia_body_inv = Mat3::Identity();  // precomputed once
};

inline RigidBodyParams make_rigid_body_params(double mass, const Mat3& inertia_body) {
    return {mass, inertia_body, inertia_body.inverse()};
}

}  // namespace apex
