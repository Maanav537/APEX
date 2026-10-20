// Demo: the mouse heads for its goal and dodges while the cat chases it with
// PN. Rerun draws the goal, both paths, each nose arrow, and the mouse's
// command (red while dodging). Sensor graphs (time series) sit beside the 3D
// view on the same sim_time timeline.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <rerun.hpp>

#include <apex/sim/simulation.hpp>

int main() {
    const apex::Vec3 start{0.0, 0.0, 1000.0};  // m
    const apex::Vec3 goal{500.0, 0.0, 1000.0};  // m
    const double mouse_speed = 20.0;            // m/s
    const double cat_speed = 50.0;              // m/s, with a 0.2 s weave the mouse gets through

    apex::RigidBodyState mouse;
    mouse.position = start;  // velocity and nose are aimed at the goal by Simulation

    apex::RigidBodyState cat;
    cat.position = apex::Vec3{600.0, 100.0, 1000.0};  // m, guarding: just past the goal
    cat.velocity = apex::Vec3{-cat_speed, 0.0, 0.0};  // flying -x, toward the mouse

    const auto params = apex::make_rigid_body_params(1.0, apex::Mat3::Identity());
    apex::Simulation sim{mouse, params, goal, mouse_speed, cat, params, cat_speed};

    const rerun::RecordingStream rec("apex_sim");
    rec.spawn().exit_on_failure();

    rec.log_static("goal", rerun::Points3D({rerun::Position3D(static_cast<float>(goal.x()),
                                                              static_cast<float>(goal.y()),
                                                              static_cast<float>(goal.z()))})
                               .with_radii({5.0f}));

    // Drawing lives outside step(): it allocates, and 100 Hz is plenty for the eye.
    const int log_every = apex::guidance_every;
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

        const apex::Vec3& p = sim.mouse().position;
        const apex::Vec3& a = sim.mouse_command();  // m/s^2, drawn 1 m per m/s^2
        const rerun::Color color =
            sim.mouse_dodging() ? rerun::Color(230, 60, 60) : rerun::Color(80, 160, 230);
        rec.log("mouse/command",
                rerun::Arrows3D::from_vectors({rerun::Vector3D(static_cast<float>(a.x()),
                                                               static_cast<float>(a.y()),
                                                               static_cast<float>(a.z()))})
                    .with_origins({rerun::Position3D(static_cast<float>(p.x()),
                                                     static_cast<float>(p.y()),
                                                     static_cast<float>(p.z()))})
                    .with_colors({color}));
    };

    // Sensor graphs: one entity per graph, several lines in it, so each graph's
    // lines always share one window. Names and colors are set once, in line order.
    const rerun::Color raw_c(170, 170, 170);
    const rerun::Color est_c(80, 160, 230);
    const rerun::Color truth_c(240, 170, 40);
    const rerun::Color limit_c(230, 60, 60);
    auto series = [&](const char* path, std::vector<rerun::components::Name> names,
                      std::vector<rerun::components::Color> colors) {
        rec.log_static(path, rerun::SeriesLines()
                                 .with_names(std::move(names))
                                 .with_colors(std::move(colors)));
    };
    series("seeker_range_error", {"raw reading - truth (m)", "r_hat - truth (m)"}, {raw_c, est_c});
    series("seeker_vrel_error", {"|v_hat - true v_rel| (m/s)"}, {est_c});
    series("mouse_sees_cat_turn", {"a_hat across LOS (m/s^2)", "cat command (m/s^2)"},
           {est_c, truth_c});
    series("mouse_t_go", {"t_go (s, capped at 20)", "dodge threshold (s)", "dodging (0/1)"},
           {est_c, limit_c, truth_c});
    series("mouse_weave", {"mouse command, up component (m/s^2)"}, {limit_c});

    // `before` is the truth the sensors read this tick: guidance runs on
    // start-of-tick truth, so compare against the state before step().
    auto log_sensors = [&](const apex::RigidBodyState& mouse_before,
                           const apex::RigidBodyState& cat_before, const apex::Vec3& cat_accel) {
        auto plot = [&](const char* path, std::vector<double> values) {
            rec.log(path, rerun::Scalars(std::move(values)));
        };

        // Cat's seeker: r = mouse - cat.
        const apex::Vec3 r_true = mouse_before.position - cat_before.position;
        const apex::Vec3 v_rel_true = mouse_before.velocity - cat_before.velocity;
        const apex::Seeker& seeker = sim.seeker();
        plot("seeker_range_error", {seeker.last_reading().norm() - r_true.norm(),
                                    seeker.r_hat().norm() - r_true.norm()});
        plot("seeker_vrel_error", {(seeker.v_hat() - v_rel_true).norm()});

        // Mouse's tracker: the turn part of a_hat (not used by the weave; shown
        // to compare the estimate with the cat's real command).
        const apex::EvadeParams evade;
        const apex::Vec3 los = (sim.tracker().p_hat() - mouse_before.position).normalized();
        const apex::Vec3& a_hat = sim.tracker().a_hat();
        plot("mouse_sees_cat_turn", {(a_hat - a_hat.dot(los) * los).norm(), cat_accel.norm()});
        plot("mouse_t_go", {std::min(sim.mouse_t_go(), 20.0), evade.t_go_on,
                            sim.mouse_dodging() ? 1.0 : 0.0});
        plot("mouse_weave", {sim.mouse_command().z()});
    };

    auto running = [&] { return !sim.ground_hit() && !sim.captured() && !sim.reached_goal(); };
    for (int i = 0; i < max_steps && running(); ++i) {
        if (i % log_every != 0) {
            sim.step();
            continue;
        }
        // Logging ticks line up with guidance ticks (both every 10th), so the
        // sensors have just updated when log_sensors reads them.
        log();
        const apex::RigidBodyState mouse_before = sim.mouse();
        const apex::RigidBodyState cat_before = sim.cat();
        const apex::Vec3 cat_accel = sim.cat_command();  // in force up to this tick
        sim.step();
        log_sensors(mouse_before, cat_before, cat_accel);
    }
    log();  // final point: capture, goal, or ground
}
