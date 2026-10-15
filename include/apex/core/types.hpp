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

}  // namespace apex
