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
                       const RigidBodyParams& cat_params, double cat_speed)
    : mouse_(mouse),
      mouse_params_(mouse_params),
      goal_(goal),
      mouse_speed_(mouse_speed),
      cat_(cat),
      cat_params_(cat_params),
      cat_speed_(cat_speed) {
    align_nose(cat_);
}

void Simulation::step() {
    if (ground_hit_ || captured_ || reached_goal_) {
        return;
    }

    // 1. Aim the mouse: velocity straight at the goal.
    mouse_.velocity = mouse_speed_ * (goal_ - mouse_.position).normalized();
    align_nose(mouse_);

    // 2. Cat guidance, 100 Hz: PN on start-of-tick truth, then limited.
    // The command is held for the next 10 ticks (and all of RK4's 4 calls).
    if (tick_ % guidance_every == 0) {
        const Vec3 r = mouse_.position - cat_.position;
        const Vec3 v_rel = mouse_.velocity - cat_.velocity;
        cat_accel_cmd_ =
            limit_accel(pn_command(r, v_rel, cat_.velocity, cat_nav_gain), cat_accel_max);
    }

    // 3. Move both. The mouse coasts; the cat is pushed by F = m * a_cmd.
    // Later, `forces` is where the atmosphere lookup, thrust, and drag go.
    const Vec3 mouse_start = mouse_.position;
    const Vec3 cat_start = cat_.position;

    auto mouse_forces = [](double, const RigidBodyState&) { return Wrench{}; };
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

    // 4. Pull q back to unit length: RK4's weighted sums drift it off the unit sphere.
    mouse_.orientation.normalize();

    // 5. Cat: the push bent its velocity; keep the new direction, reset the
    // speed (a finite step grows |v| slightly), and point the nose along it.
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
