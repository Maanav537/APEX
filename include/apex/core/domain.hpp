#pragma once

#include <apex/core/types.hpp>

namespace apex {

using PursuerState = PointMassState;
using EvaderState = PointMassState;

// Outcome of checking terminal-state convergence (3D rendezvous).
enum class ConvergenceStatus {
    InProgress,
    Converged,
    Diverged,
};

// Terminal boundary condition: relative range below this value counts as
// rendezvous (ZEM minimization success for the current step).
struct ProximityThreshold {
    double meters = 0.0;
};

// Relative geometry for a LOS bearing / relative-state observer.
struct RelativeState {
    Vec3 los = Vec3::Zero();           // evader_pos - pursuer_pos
    double range_m = 0.0;              // ||los||
    Vec3 closing_velocity = Vec3::Zero();  // pursuer_vel - evader_vel
};

inline RelativeState relative_state(const Vec3& pursuer_pos,
                                    const Vec3& pursuer_vel,
                                    const Vec3& evader_pos,
                                    const Vec3& evader_vel) {
    RelativeState r;
    r.los = evader_pos - pursuer_pos;
    r.range_m = r.los.norm();
    r.closing_velocity = pursuer_vel - evader_vel;
    return r;
}

inline RelativeState relative_state(const PursuerState& pursuer,
                                    const EvaderState& evader) {
    return relative_state(pursuer.position, pursuer.velocity,
                          evader.position, evader.velocity);
}

// Classify terminal-state convergence against a proximity threshold.
inline ConvergenceStatus evaluate_convergence(double range_m,
                                              ProximityThreshold threshold) {
    if (!(threshold.meters > 0.0) || !(range_m >= 0.0)) {
        return ConvergenceStatus::Diverged;
    }
    if (range_m <= threshold.meters) {
        return ConvergenceStatus::Converged;
    }
    return ConvergenceStatus::InProgress;
}

inline ConvergenceStatus evaluate_convergence(const Vec3& pursuer_pos,
                                              const Vec3& evader_pos,
                                              ProximityThreshold threshold) {
    return evaluate_convergence((evader_pos - pursuer_pos).norm(), threshold);
}

}  // namespace apex
