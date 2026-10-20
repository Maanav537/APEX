#include <apex/sim/simulation.hpp>

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

}  // namespace

Simulation::Simulation(const RigidBodyState& mouse, const RigidBodyParams& mouse_params,
                       const Vec3& goal, double mouse_speed, const RigidBodyState& cat,
                       const RigidBodyParams& cat_params, double cat_speed,
                       const SeekerParams& seeker_params, const TrackerParams& tracker_params,
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
      seeker_(seeker_params) {
    mouse_.velocity = mouse_speed_ * (goal_ - mouse_.position).normalized();
    align_nose(mouse_);
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
    }

    // 2. Cat guidance, 100 Hz: the seeker reads start-of-tick truth (with noise)
    // and filters it; PN runs on its estimates, then is limited. v_cat is the
    // cat's own state, not a seeker output. The command is held for the next
    // 10 ticks (and all of RK4's 4 calls).
    if (tick_ % guidance_every == 0) {
        seeker_.update(mouse_.position - cat_.position);
        cat_accel_cmd_ = limit_accel(
            pn_command(seeker_.r_hat(), seeker_.v_hat(), cat_.velocity, cat_nav_gain),
            cat_accel_max);
    }

    // 3. Move both, each pushed by F = m * a_cmd.
    // Later, `forces` is where the atmosphere lookup, thrust, and drag go.
    const Vec3 mouse_start = mouse_.position;
    const Vec3 cat_start = cat_.position;

    const Wrench mouse_push{mouse_params_.mass * mouse_cmd_.accel, Vec3::Zero()};
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

    // 4-5. The push bent each velocity; keep the new direction, reset the speed
    // (a finite step grows |v| slightly), and point the nose along it. That also
    // rebuilds q as a unit quaternion, so no separate normalize is needed.
    mouse_.velocity = mouse_speed_ * mouse_.velocity.normalized();
    align_nose(mouse_);
    cat_.velocity = cat_speed_ * cat_.velocity.normalized();
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
