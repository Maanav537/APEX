#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <apex/sim/capture.hpp>

using apex::closest_distance;
using apex::Vec3;
using Catch::Matchers::WithinAbs;

TEST_CASE("catches a pass that happens between ticks", "[capture]") {
    // Gap swings from 5 m on one side to 5 m on the other, 0.1 m off-axis.
    // Both ends are far outside the radius; the middle of the tick is not.
    const Vec3 r0{5.0, 0.1, 0.0};
    const Vec3 r1{-5.0, 0.1, 0.0};
    REQUIRE_THAT(closest_distance(r0, r1), WithinAbs(0.1, 1e-12));
    REQUIRE(closest_distance(r0, r1) <= apex::capture_radius);
}

TEST_CASE("moving apart: closest is the start of the tick (s* < 0)", "[capture]") {
    const Vec3 r0{2.0, 0.0, 0.0};
    const Vec3 r1{3.0, 0.0, 0.0};
    REQUIRE_THAT(closest_distance(r0, r1), WithinAbs(2.0, 1e-12));
}

TEST_CASE("still closing: closest is the end of the tick (s* > 1)", "[capture]") {
    const Vec3 r0{3.0, 0.0, 0.0};
    const Vec3 r1{2.0, 0.0, 0.0};
    REQUIRE_THAT(closest_distance(r0, r1), WithinAbs(2.0, 1e-12));
}

TEST_CASE("constant gap (d = 0) gives |r0|, not NaN", "[capture]") {
    const Vec3 r{3.0, 4.0, 0.0};
    REQUIRE_THAT(closest_distance(r, r), WithinAbs(5.0, 1e-12));
}
