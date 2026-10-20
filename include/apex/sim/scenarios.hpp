#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include <apex/sim/simulation.hpp>

namespace apex {

// A preset starting setup. Every field is something the Simulation constructor
// already takes, so scenarios change no engine code.
struct Scenario {
    std::string id;
    std::string name;
    std::string description;

    Vec3 mouse_start;
    Vec3 goal;
    double mouse_speed = 20.0;  // m/s, cruise
    Vec3 cat_start;
    Vec3 cat_heading;          // only the direction is used
    double cat_speed = 50.0;   // m/s, cruise

    CatObserverParams observer;
    TrackerParams tracker;
    EvadeParams evade;
};

struct ScenarioInfo {
    const char* id;
    const char* name;
    const char* description;
};

inline const std::vector<ScenarioInfo>& scenario_list() {
    static const std::vector<ScenarioInfo> list{
        {"basic", "Basic", "Mouse flies 500 m to its goal; the cat guards just past the goal."},
        {"random", "Random spawn",
         "Basic, but the mouse starts at a random spot in a 300 m flat circle (set by the seed)."},
        {"tail_chase", "Tail chase", "The cat starts 400 m behind the mouse and has to run it down."},
        {"side", "Side intercept", "The cat comes in from 400 m off to the side."},
        {"no_dodge", "No dodge",
         "Basic with the weave turned off: a baseline to compare dodging against."},
        {"noisy", "Noisy sensors", "Basic with both sensors 10x noisier (range and angle)."},
    };
    return list;
}

// Flat circle (same altitude) around `center`. sqrt(u) spreads points evenly over
// the area instead of bunching them at the center.
inline Vec3 random_spawn(const Vec3& center, std::uint32_t seed) {
    constexpr double spawn_radius = 300.0;  // m, keeps the mouse at least 200 m from the goal
    constexpr double pi = 3.14159265358979323846;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double radius = spawn_radius * std::sqrt(unit(rng));
    const double angle = 2.0 * pi * unit(rng);
    return center + Vec3{radius * std::cos(angle), radius * std::sin(angle), 0.0};
}

// Builds the scenario `id`, or nothing if the id is unknown. `seed` picks the
// random spawn. Sensor noise seeds stay fixed (so runs repeat) unless `vary_noise`.
inline std::optional<Scenario> make_scenario(std::string_view id, std::uint32_t seed = 1,
                                             bool vary_noise = false) {
    const ScenarioInfo* info = nullptr;
    for (const ScenarioInfo& s : scenario_list()) {
        if (id == s.id) {
            info = &s;
        }
    }
    if (info == nullptr) {
        return std::nullopt;
    }

    // Basic demo; the others start from it.
    Scenario sc;
    sc.id = info->id;
    sc.name = info->name;
    sc.description = info->description;
    sc.mouse_start = Vec3{0.0, 0.0, 1000.0};       // m
    sc.goal = Vec3{500.0, 0.0, 1000.0};            // m
    sc.cat_start = Vec3{600.0, 100.0, 1000.0};     // m, guarding: just past the goal
    sc.cat_heading = Vec3{-1.0, 0.0, 0.0};         // flying -x, toward the mouse

    if (id == "random") {
        sc.mouse_start = random_spawn(sc.mouse_start, seed);
    } else if (id == "tail_chase") {
        sc.cat_start = Vec3{-400.0, 0.0, 1000.0};
        sc.cat_heading = Vec3{1.0, 0.0, 0.0};
    } else if (id == "side") {
        sc.cat_start = Vec3{250.0, 400.0, 1000.0};
        sc.cat_heading = Vec3{0.0, -1.0, 0.0};
    } else if (id == "no_dodge") {
        sc.evade.t_go_on = 0.0;  // t_go is never below 0, so the weave never starts
    } else if (id == "noisy") {
        sc.observer.range_sigma *= 10.0;
        sc.observer.angle_sigma *= 10.0;
        sc.tracker.range_sigma *= 10.0;
        sc.tracker.angle_sigma *= 10.0;
    }

    if (vary_noise) {
        sc.observer.seed = seed * 2u + 1u;
        sc.tracker.seed = seed * 2u + 2u;
    }
    return sc;
}

inline Simulation make_simulation(const Scenario& sc) {
    RigidBodyState mouse;
    mouse.position = sc.mouse_start;  // velocity and nose are aimed at the goal by Simulation
    RigidBodyState cat;
    cat.position = sc.cat_start;
    cat.velocity = sc.cat_speed * sc.cat_heading.normalized();
    const auto params = make_rigid_body_params(1.0, Mat3::Identity());
    return Simulation{mouse,  params, sc.goal,     sc.mouse_speed, cat,     params,
                      sc.cat_speed, sc.observer, sc.tracker, sc.evade};
}

// Upper time limit, s: twice the mouse's straight-line flight time.
inline double scenario_max_time(const Scenario& sc) {
    return 2.0 * (sc.goal - sc.mouse_start).norm() / sc.mouse_speed;
}

}  // namespace apex
