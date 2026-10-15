#pragma once

#include <apex/core/types.hpp>

namespace apex {

// Fixed step: 1000 Hz.
inline constexpr double sim_dt = 0.001;  // s

// The world, advanced one tick at a time. Axes: z is up, the ground is z = 0.
//
// Week 1 holds only the dummy mouse, pulled toward `center` so it flies a
// circle (a = -omega^2 * (position - center)). No air is needed yet.
class Simulation {
public:
    Simulation(const PointMassState& mouse, const Vec3& center, double omega);

    // One tick: RK4 move, advance the clock, check the floor. Allocates nothing.
    // Does nothing once the mouse has hit the ground.
    void step();

    const PointMassState& mouse() const { return mouse_; }
    double time() const { return t_; }
    bool ground_hit() const { return ground_hit_; }

private:
    PointMassState mouse_;
    Vec3 center_;
    double omega_;
    double t_ = 0.0;
    bool ground_hit_ = false;
};

}  // namespace apex
