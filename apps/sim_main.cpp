// Demo: the mouse flies a straight line to its goal while the cat chases it
// with PN. Rerun draws the goal, both paths, and each nose arrow.

#include <cmath>
#include <string>
#include <vector>

#include <rerun.hpp>

#include <apex/sim/simulation.hpp>

int main() {
    const apex::Vec3 start{0.0, 0.0, 1000.0};  // m
    const apex::Vec3 goal{500.0, 0.0, 1000.0};  // m
    const double mouse_speed = 20.0;            // m/s
    const double cat_speed = 60.0;              // m/s

    apex::RigidBodyState mouse;
    mouse.position = start;  // velocity and nose are aimed by step() each tick

    apex::RigidBodyState cat;
    cat.position = apex::Vec3{-200.0, -300.0, 1000.0};  // m, off to the side and behind
    cat.velocity = apex::Vec3{cat_speed, 0.0, 0.0};     // flying +x

    const auto params = apex::make_rigid_body_params(1.0, apex::Mat3::Identity());
    apex::Simulation sim{mouse, params, goal, mouse_speed, cat, params, cat_speed};

    const rerun::RecordingStream rec("apex_sim");
    rec.spawn().exit_on_failure();

    rec.log_static("goal", rerun::Points3D({rerun::Position3D(static_cast<float>(goal.x()),
                                                              static_cast<float>(goal.y()),
                                                              static_cast<float>(goal.z()))})
                               .with_radii({5.0f}));

    // Drawing lives outside step(): it allocates, and 100 Hz is plenty for the eye.
    const int log_every = 10;
    const double max_time = 2.0 * (goal - start).norm() / mouse_speed;  // s, safety stop
    const int max_steps = static_cast<int>(std::lround(max_time / apex::sim_dt));
    const double nose_length = 20.0;  // m, display only

    std::vector<rerun::datatypes::Vec3D> mouse_path;
    std::vector<rerun::datatypes::Vec3D> cat_path;
    mouse_path.reserve(max_steps / log_every + 2);
    cat_path.reserve(max_steps / log_every + 2);

    auto log_body = [&](const std::string& name, const apex::RigidBodyState& x,
                        std::vector<rerun::datatypes::Vec3D>& path) {
        const apex::Vec3 nose = nose_length * (x.orientation * apex::Vec3::UnitX());
        const rerun::datatypes::Vec3D point{static_cast<float>(x.position.x()),
                                            static_cast<float>(x.position.y()),
                                            static_cast<float>(x.position.z())};
        path.push_back(point);

        rec.log(name, rerun::Points3D({rerun::Position3D(point.x(), point.y(), point.z())})
                          .with_radii({3.0f}));
        rec.log(name + "/nose",
                rerun::Arrows3D::from_vectors({rerun::Vector3D(static_cast<float>(nose.x()),
                                                               static_cast<float>(nose.y()),
                                                               static_cast<float>(nose.z()))})
                    .with_origins({rerun::Position3D(point.x(), point.y(), point.z())}));
        rec.log(name + "/path", rerun::LineStrips3D(rerun::components::LineStrip3D(path)));
    };

    auto log = [&] {
        rec.set_time_duration_secs("sim_time", sim.time());
        log_body("mouse", sim.mouse(), mouse_path);
        log_body("cat", sim.cat(), cat_path);
    };

    auto running = [&] { return !sim.ground_hit() && !sim.captured() && !sim.reached_goal(); };
    for (int i = 0; i < max_steps && running(); ++i) {
        if (i % log_every == 0) {
            log();
        }
        sim.step();
    }
    log();  // final point: capture, goal, or ground
}
