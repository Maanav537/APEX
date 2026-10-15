#include <apex/sim/simulation.hpp>

#include <apex/physics/rk4.hpp>

namespace apex {

Simulation::Simulation(const PointMassState& mouse, const Vec3& center, double omega)
    : mouse_(mouse), center_(center), omega_(omega) {}

void Simulation::step() {
    if (ground_hit_) {
        return;
    }

    // 1. Move. Later, `f` is where the atmosphere lookup and forces go.
    auto f = [this](double, const PointMassState& x) {
        return PointMassState{x.velocity, -omega_ * omega_ * (x.position - center_)};
    };
    mouse_ = rk4_step(f, mouse_, t_, sim_dt);

    // 2. Advance the clock (after the move: RK4 needed the start-of-tick time).
    t_ += sim_dt;

    // 3. Floor check: touching the ground loses.
    if (mouse_.position.z() <= 0.0) {
        ground_hit_ = true;
    }
}

}  // namespace apex
