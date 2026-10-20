#pragma once

#include <apex/core/types.hpp>
#include <apex/gnc/evade.hpp>
#include <apex/gnc/seeker.hpp>
#include <apex/gnc/tracker.hpp>

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
// The mouse heads for a fixed `goal` and weaves to dodge the cat (see evade_command),
// fed by its tracker (noisy reading + alpha-beta-gamma filter on the cat's
// position), not by truth. It starts flying at the goal at `mouse_speed`.
//
// The cat chases it with PN fed by its seeker (noisy reading + alpha-beta
// filter), not by truth. `cat` must start with nonzero velocity.
//
// Both are moved the same way: each limited command becomes a force F = m * a,
// RK4 integrates it, then the speed is reset to constant and the nose (body +x)
// is set along the velocity. Both commands update at 100 Hz and are held between.
class Simulation {
public:
    Simulation(const RigidBodyState& mouse, const RigidBodyParams& mouse_params, const Vec3& goal,
               double mouse_speed, const RigidBodyState& cat, const RigidBodyParams& cat_params,
               double cat_speed, const SeekerParams& seeker_params = {},
               const TrackerParams& tracker_params = {}, const EvadeParams& evade_params = {});

    // One tick: update both commands (every 10th tick), RK4 both, reset speeds
    // and noses, advance the clock, then check floor, capture, goal. Allocates nothing. Does nothing once any of those ends it.
    void step();

    const RigidBodyState& mouse() const { return mouse_; }
    const RigidBodyState& cat() const { return cat_; }
    const Vec3& goal() const { return goal_; }
    const Vec3& cat_command() const { return cat_accel_cmd_; }  // m/s^2, world frame, limited
    const Seeker& seeker() const { return seeker_; }
    const Vec3& mouse_command() const { return mouse_cmd_.accel; }  // m/s^2, world frame, limited
    bool mouse_dodging() const { return mouse_cmd_.dodging; }
    double mouse_t_go() const { return mouse_cmd_.t_go; }  // s, from the tracker's estimates
    const Tracker& tracker() const { return tracker_; }
    double time() const { return t_; }
    bool ground_hit() const { return ground_hit_; }
    bool captured() const { return captured_; }
    bool reached_goal() const { return reached_goal_; }

private:
    RigidBodyState mouse_;
    RigidBodyParams mouse_params_;
    Vec3 goal_;
    double mouse_speed_;
    EvadeParams evade_params_;
    EvadeCommand mouse_cmd_;  // held between guidance updates
    bool weaving_ = false;
    double weave_start_ = 0.0;  // s, when the current weave began
    Tracker tracker_;

    RigidBodyState cat_;
    RigidBodyParams cat_params_;
    double cat_speed_;
    Vec3 cat_accel_cmd_ = Vec3::Zero();  // held between guidance updates
    Seeker seeker_;

    double t_ = 0.0;
    long long tick_ = 0;
    bool ground_hit_ = false;
    bool captured_ = false;
    bool reached_goal_ = false;
};

}  // namespace apex
