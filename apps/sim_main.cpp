// Week 1 demo: the dummy mouse flies one full circle while Rerun draws its path.

#include <cmath>
#include <numbers>
#include <vector>

#include <rerun.hpp>

#include <apex/sim/simulation.hpp>

int main() {
    const double r = 100.0;          // m
    const double omega = 0.5;        // rad/s
    const double altitude = 1000.0;  // m
    const apex::Vec3 center{0.0, 0.0, altitude};

    apex::Simulation sim{{center + apex::Vec3{r, 0.0, 0.0}, apex::Vec3{0.0, r * omega, 0.0}},
                         center, omega};

    const rerun::RecordingStream rec("apex_sim");
    rec.spawn().exit_on_failure();

    // Drawing lives outside step(): it allocates, and 100 Hz is plenty for the eye.
    const int log_every = 10;
    const double period = 2.0 * std::numbers::pi / omega;
    const int steps = static_cast<int>(std::lround(period / apex::sim_dt));

    std::vector<rerun::datatypes::Vec3D> path;
    path.reserve(steps / log_every + 2);

    auto log = [&] {
        const apex::Vec3& p = sim.mouse().position;
        const rerun::datatypes::Vec3D point{static_cast<float>(p.x()), static_cast<float>(p.y()),
                                            static_cast<float>(p.z())};
        path.push_back(point);

        rec.set_time_duration_secs("sim_time", sim.time());
        rec.log("mouse", rerun::Points3D({rerun::Position3D(point.x(), point.y(), point.z())})
                             .with_radii({3.0f}));
        rec.log("mouse/path", rerun::LineStrips3D(rerun::components::LineStrip3D(path)));
    };

    for (int i = 0; i < steps && !sim.ground_hit(); ++i) {
        if (i % log_every == 0) {
            log();
        }
        sim.step();
    }
    log();  // final point closes the circle
}
