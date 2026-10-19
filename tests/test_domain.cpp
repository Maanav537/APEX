#include <catch2/catch_test_macros.hpp>

#include <apex/core/domain.hpp>

using apex::ConvergenceStatus;
using apex::EvaderState;
using apex::ProximityThreshold;
using apex::PursuerState;
using apex::Vec3;

TEST_CASE("relative_state uses pursuer_pos and evader_pos", "[domain]") {
    const Vec3 pursuer_pos{0.0, 0.0, 0.0};
    const Vec3 pursuer_vel{1.0, 0.0, 0.0};
    const Vec3 evader_pos{3.0, 4.0, 0.0};
    const Vec3 evader_vel{0.0, 0.0, 0.0};

    const auto rel =
        apex::relative_state(pursuer_pos, pursuer_vel, evader_pos, evader_vel);

    REQUIRE(rel.los == Vec3{3.0, 4.0, 0.0});
    REQUIRE(rel.range_m == 5.0);
    REQUIRE(rel.closing_velocity == Vec3{1.0, 0.0, 0.0});
}

TEST_CASE("relative_state accepts PursuerState and EvaderState", "[domain]") {
    const PursuerState pursuer{{1.0, 2.0, 3.0}, {0.0, 1.0, 0.0}};
    const EvaderState evader{{1.0, 2.0, 6.0}, {0.0, 0.0, 0.0}};

    const auto rel = apex::relative_state(pursuer, evader);
    REQUIRE(rel.range_m == 3.0);
    REQUIRE(rel.closing_velocity == Vec3{0.0, 1.0, 0.0});
}

TEST_CASE("evaluate_convergence reports ConvergenceStatus", "[domain]") {
    const ProximityThreshold threshold{2.0};
    const Vec3 pursuer_pos{0.0, 0.0, 0.0};

    REQUIRE(apex::evaluate_convergence(pursuer_pos, Vec3{1.0, 0.0, 0.0},
                                       threshold) ==
            ConvergenceStatus::Converged);
    REQUIRE(apex::evaluate_convergence(pursuer_pos, Vec3{5.0, 0.0, 0.0},
                                       threshold) ==
            ConvergenceStatus::InProgress);
    REQUIRE(apex::evaluate_convergence(-1.0, threshold) ==
            ConvergenceStatus::Diverged);
    REQUIRE(apex::evaluate_convergence(1.0, ProximityThreshold{0.0}) ==
            ConvergenceStatus::Diverged);
}
