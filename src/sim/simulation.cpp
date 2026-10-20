#include <apex/sim/simulation.hpp>

#include <algorithm>

#include <apex/gnc/pn.hpp>
#include <apex/physics/rigid_body.hpp>
#include <apex/physics/rk4.hpp>
#include <apex/sim/capture.hpp>

namespace apex {

namespace {

// Nose (body +x) along the velocity, no spin.
void align_nose(RigidBodyState& x) {
    x.orientation = Quat::FromTwoVectors(Vec3::UnitX(), x.velocity.normalized());
    x.angular_velocity = Vec3::Zero();
}

// Guidance period, s: commands are held this long.
constexpr double guidance_dt = guidance_every * sim_dt;

// Turn push + speed push toward the cruise or juke speed, limited together (option C).
Vec3 command_with_speed(const Vec3& turn, const Vec3& velocity, double cruise, bool juking,
                        double a_max, const SpeedParams& sp) {
    const double speed = velocity.norm();
    const double target = cruise * (juking ? sp.juke_fraction : 1.0);
    const double push = speed_command(speed, target, guidance_dt, sp.accel_limit);
    return limit_turn_and_speed(turn, push, velocity / speed, a_max, sp.accel_limit,
                                sp.accel_limit);
}

// Keep the speed in [min, cruise] (a finite step can push it slightly past).
void clamp_speed(Vec3& velocity, double cruise, const SpeedParams& sp) {
    const double speed = velocity.norm();
    velocity *= std::clamp(speed, sp.min_fraction * cruise, cruise) / speed;
}

}  // namespace

Simulation::Simulation(const RigidBodyState& mouse, const RigidBodyParams& mouse_params,
                       const Vec3& goal, double mouse_speed, const RigidBodyState& cat,
                       const RigidBodyParams& cat_params, double cat_speed,
                       const CatObserverParams& observer_params, const TrackerParams& tracker_params,
                       const EvadeParams& evade_params)
    : mouse_(mouse),
      mouse_params_(mouse_params),
      goal_(goal),
      mouse_speed_(mouse_speed),
      evade_params_(evade_params),
      tracker_(tracker_params),
      cat_(cat),
      cat_params_(cat_params),
      cat_speed_(cat_speed),
      observer_(observer_params) {
    mouse_.velocity = mouse_speed_ * (goal_ - mouse_.position).normalized();
    align_nose(mouse_);
    cat_.velocity = cat_speed_ * cat_.velocity.normalized();
    align_nose(cat_);
}

void Simulation::step() {
    if (ground_hit_ || captured_ || reached_goal_) {
        return;
    }

    // 1. Both guidance laws, 100 Hz, on start-of-tick truth read through noisy
    // sensors. Commands are held for the next 10 ticks (and all of RK4's 4 calls).
    if (tick_ % guidance_every == 0) {
        // Mouse: its tracker estimates the cat in world terms; the mouse knows
        // its own position and velocity exactly, so it subtracts them here.
        // The weave clock starts on the first dodging update and resets if it stops.
        tracker_.update(mouse_.position, cat_.position - mouse_.position);
        const double time_in_weave = weaving_ ? t_ - weave_start_ : 0.0;
        mouse_cmd_ = evade_command(tracker_.p_hat() - mouse_.position,
                                   tracker_.v_hat() - mouse_.velocity, mouse_.velocity,
                                   goal_ - mouse_.position, time_in_weave, evade_params_);
        if (mouse_cmd_.dodging && !weaving_) {
            weave_start_ = t_;
        }
        weaving_ = mouse_cmd_.dodging;
        mouse_accel_cmd_ = command_with_speed(mouse_cmd_.accel, mouse_.velocity, mouse_speed_,
                                              weaving_, evade_params_.accel_max,
                                              mouse_speed_params);
    }

    // 2. Cat guidance, 100 Hz: the observer reads start-of-tick truth (with noise)
    // and filters the mouse's world state; the cat subtracts its own known position
    // and velocity, then pursuit runs on those estimates. If a_hat looks like a
    // weave, the cat slows to its juke speed. The command is held for the next 10
    // ticks (and all of RK4's 4 calls).
    if (tick_ % guidance_every == 0) {
        observer_.update(cat_.position, mouse_.position - cat_.position, cat_.velocity);
        const Vec3 r_hat = observer_.p_hat() - cat_.position;
        cat_sees_weave_ = weave_detector_.update(t_, observer_.a_hat(), r_hat);
        const Vec3 turn = pursuit_command(r_hat, observer_.v_hat() - cat_.velocity,
                                          cat_.velocity, cat_nav_gain);
        cat_accel_cmd_ = command_with_speed(turn, cat_.velocity, cat_speed_, cat_sees_weave_,
                                            cat_accel_max, SpeedParams{});
    }

    // 3. Move both, each pushed by F = m * a_cmd.
    // Later, `forces` is where the atmosphere lookup, thrust, and drag go.
    const Vec3 mouse_start = mouse_.position;
    const Vec3 cat_start = cat_.position;

    const Wrench mouse_push{mouse_params_.mass * mouse_accel_cmd_, Vec3::Zero()};
    auto mouse_forces = [&](double, const RigidBodyState&) { return mouse_push; };
    auto mouse_f = [&](double t, const RigidBodyState& x) {
        return rigid_body_derivative(t, x, mouse_params_, mouse_forces);
    };
    mouse_ = rk4_step(mouse_f, mouse_, t_, sim_dt);

    const Wrench cat_push{cat_params_.mass * cat_accel_cmd_, Vec3::Zero()};
    auto cat_forces = [&](double, const RigidBodyState&) { return cat_push; };
    auto cat_f = [&](double t, const RigidBodyState& x) {
        return rigid_body_derivative(t, x, cat_params_, cat_forces);
    };
    cat_ = rk4_step(cat_f, cat_, t_, sim_dt);

    // 4-5. The push bent and sped up or slowed each velocity; keep its speed in
    // [min, cruise] and point the nose along it. That also rebuilds q as a unit
    // quaternion, so no separate normalize is needed.
    clamp_speed(mouse_.velocity, mouse_speed_, mouse_speed_params);
    align_nose(mouse_);
    clamp_speed(cat_.velocity, cat_speed_, SpeedParams{});
    align_nose(cat_);

    // 6. Advance the clock (after the move: RK4 needed the start-of-tick time).
    t_ += sim_dt;
    ++tick_;

    // 7. Floor check: the mouse touching the ground loses.
    if (mouse_.position.z() <= 0.0) {
        ground_hit_ = true;
        return;
    }

    // 8. Capture over the whole tick, checked before the goal: a tie goes to the cat.
    if (closest_distance(mouse_start - cat_start, mouse_.position - cat_.position) <=
        capture_radius) {
        captured_ = true;
        return;
    }

    // 9. Goal check over the whole tick: the cat loses.
    if (closest_distance(goal_ - mouse_start, goal_ - mouse_.position) <= goal_radius) {
        reached_goal_ = true;
    }
}

}  // namespace apex
