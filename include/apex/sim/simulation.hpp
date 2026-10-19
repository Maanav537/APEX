#pragma once

#include <apex/core/types.hpp>

namespace apex {

// Fixed step: 1000 Hz.
inline constexpr double sim_dt = 0.001;  // s

// Cat guidance: PN runs once every `guidance_every` ticks (100 Hz) and its
// command is held in between.
inline constexpr int guidance_every = 10;
inline constexpr double cat_nav_gain = 5.0;   // N, unitless
inline constexpr double cat_accel_max = 100.0;  // m/s^2

// The world, advanced one tick at a time. Axes: z is up, the ground is z = 0.
//
// The mouse flies a straight line to a fixed `goal` at a constant speed: its
// velocity is set each tick, not pushed by a force.
//
// The cat chases it with PN: the limited command becomes a force F = m * a,
// RK4 integrates it, then the cat's speed is reset to `cat_speed` and its nose
// (body +x) is set along its velocity. `cat` must start with nonzero velocity.
class Simulation {
public:
    Simulation(const RigidBodyState& mouse, const RigidBodyParams& mouse_params, const Vec3& goal,
               double mouse_speed, const RigidBodyState& cat, const RigidBodyParams& cat_params,
               double cat_speed);

    // One tick: aim the mouse, update the cat's command (every 10th tick), RK4
    // both, normalize q, reset cat speed and nose, advance the clock, then check
    // floor, capture, goal. Allocates nothing. Does nothing once any of those ends it.
    void step();

    const RigidBodyState& mouse() const { return mouse_; }
    const RigidBodyState& cat() const { return cat_; }
    const Vec3& goal() const { return goal_; }
    const Vec3& cat_command() const { return cat_accel_cmd_; }  // m/s^2, world frame, limited
    double time() const { return t_; }
    bool ground_hit() const { return ground_hit_; }
    bool captured() const { return captured_; }
    bool reached_goal() const { return reached_goal_; }

private:
    RigidBodyState mouse_;
    RigidBodyParams mouse_params_;
    Vec3 goal_;
    double mouse_speed_;

    RigidBodyState cat_;
    RigidBodyParams cat_params_;
    double cat_speed_;
    Vec3 cat_accel_cmd_ = Vec3::Zero();  // held between guidance updates

    double t_ = 0.0;
    long long tick_ = 0;
    bool ground_hit_ = false;
    bool captured_ = false;
    bool reached_goal_ = false;
};

}  // namespace apex
