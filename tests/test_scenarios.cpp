#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <random>

#include <apex/sim/scenarios.hpp>

namespace {

// Steps until the run ends or the scenario's time limit; returns the sim.
apex::Simulation run_to_end(const apex::Scenario& sc) {
    apex::Simulation sim = apex::make_simulation(sc);
    const int max_steps =
        static_cast<int>(std::lround(apex::scenario_max_time(sc) / apex::sim_dt));
    for (int i = 0; i < max_steps && !sim.captured() && !sim.reached_goal() && !sim.ground_hit();
         ++i) {
        sim.step();
    }
    return sim;
}

}  // namespace

TEST_CASE("every scenario builds and runs to an end state", "[scenarios]") {
    for (const apex::ScenarioInfo& info : apex::scenario_list()) {
        INFO(info.id);
        const auto sc = apex::make_scenario(info.id, 7);
        REQUIRE(sc);
        const apex::Simulation sim = run_to_end(*sc);
        CHECK((sim.captured() || sim.reached_goal() || sim.ground_hit()));
    }
}

TEST_CASE("unknown scenario id gives nothing", "[scenarios]") {
    CHECK_FALSE(apex::make_scenario("nope"));
}

TEST_CASE("basic scenario matches the hand-built demo", "[scenarios]") {
    // The demo as apex_sim built it before scenarios existed.
    apex::RigidBodyState mouse;
    mouse.position = apex::Vec3{0.0, 0.0, 1000.0};
    apex::RigidBodyState cat;
    cat.position = apex::Vec3{600.0, 100.0, 1000.0};
    cat.velocity = apex::Vec3{-50.0, 0.0, 0.0};
    const auto params = apex::make_rigid_body_params(1.0, apex::Mat3::Identity());
    apex::Simulation by_hand{mouse, params, apex::Vec3{500.0, 0.0, 1000.0}, 20.0, cat, params,
                             50.0};
    while (!by_hand.captured() && !by_hand.reached_goal() && !by_hand.ground_hit() &&
           by_hand.time() < 50.0) {
        by_hand.step();
    }

    const apex::Simulation from_scenario = run_to_end(*apex::make_scenario("basic"));
    CHECK(from_scenario.captured() == by_hand.captured());
    CHECK(from_scenario.reached_goal() == by_hand.reached_goal());
    CHECK(from_scenario.time() == by_hand.time());
    CHECK(from_scenario.mouse().position == by_hand.mouse().position);
}

TEST_CASE("random spawn repeats per seed and stays in the circle", "[scenarios]") {
    const apex::Vec3 center{0.0, 0.0, 1000.0};
    const apex::Vec3 a = apex::make_scenario("random", 3)->mouse_start;
    CHECK(a == apex::make_scenario("random", 3)->mouse_start);
    CHECK(a != apex::make_scenario("random", 4)->mouse_start);

    for (std::uint32_t seed = 0; seed < 200; ++seed) {
        const apex::Vec3 p = apex::make_scenario("random", seed)->mouse_start;
        CHECK((p - center).norm() <= 300.0);
        CHECK(p.z() == 1000.0);
    }
}

TEST_CASE("noise seeds stay fixed unless vary_noise is set", "[scenarios]") {
    const apex::CatObserverParams defaults;
    CHECK(apex::make_scenario("basic", 9)->observer.seed == defaults.seed);
    CHECK(apex::make_scenario("basic", 9, true)->observer.seed != defaults.seed);
}
