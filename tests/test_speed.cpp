#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <apex/gnc/pn.hpp>
#include <apex/gnc/speed.hpp>

using apex::limit_turn_and_speed;
using apex::speed_command;
using apex::Vec3;
using apex::WeaveDetector;
using Catch::Matchers::WithinAbs;

namespace {
const Vec3 u_x = Vec3::UnitX();  // flying +x
}

TEST_CASE("limiter: small turn and speed push pass through unchanged", "[speed]") {
    const Vec3 a = limit_turn_and_speed(Vec3{0.0, 30.0, 0.0}, -8.0, u_x, 100.0, 10.0, 10.0);
    REQUIRE((a - Vec3{-8.0, 30.0, 0.0}).norm() < 1e-12);
}

TEST_CASE("limiter: drops the part of the turn along the velocity", "[speed]") {
    const Vec3 a = limit_turn_and_speed(Vec3{40.0, 30.0, 0.0}, 0.0, u_x, 100.0, 10.0, 10.0);
    REQUIRE((a - Vec3{0.0, 30.0, 0.0}).norm() < 1e-12);
}

TEST_CASE("limiter: a full turn still leaves the reserve for braking", "[speed]") {
    // Turn asks for 200: cut to sqrt(100^2 - 10^2) so the 10 m/s^2 brake fits.
    const Vec3 a = limit_turn_and_speed(Vec3{0.0, 200.0, 0.0}, -50.0, u_x, 100.0, 10.0, 10.0);
    REQUIRE_THAT(a.y(), WithinAbs(std::sqrt(100.0 * 100.0 - 10.0 * 10.0), 1e-9));
    REQUIRE_THAT(a.x(), WithinAbs(-10.0, 1e-12));
    REQUIRE_THAT(a.norm(), WithinAbs(100.0, 1e-9));  // right angles: shares the cap exactly
}

TEST_CASE("limiter: turning comes first, speed gets the leftover above the reserve", "[speed]") {
    // Speed limit 30 but reserve 10: a 99 turn leaves sqrt(100^2 - 99^2) ~ 14.1 for speed.
    const Vec3 a = limit_turn_and_speed(Vec3{0.0, 99.0, 0.0}, 30.0, u_x, 100.0, 30.0, 10.0);
    REQUIRE_THAT(a.y(), WithinAbs(99.0, 1e-12));
    REQUIRE_THAT(a.x(), WithinAbs(std::sqrt(100.0 * 100.0 - 99.0 * 99.0), 1e-9));
}

TEST_CASE("speed command reaches the target in one period, else is limited", "[speed]") {
    REQUIRE_THAT(speed_command(50.0, 49.95, 0.01, 10.0), WithinAbs(-5.0, 1e-9));  // exact
    REQUIRE_THAT(speed_command(50.0, 40.0, 0.01, 10.0), WithinAbs(-10.0, 1e-12));  // brake
    REQUIRE_THAT(speed_command(40.0, 50.0, 0.01, 10.0), WithinAbs(10.0, 1e-12));   // throttle
    REQUIRE(speed_command(50.0, 50.0, 0.01, 10.0) == 0.0);
}

TEST_CASE("weave detector: only the part of a_hat across the LOS counts", "[speed]") {
    WeaveDetector d;
    const Vec3 r{300.0, 0.0, 0.0};
    REQUIRE_FALSE(d.update(0.0, Vec3{50.0, 0.0, 0.0}, r));  // along the LOS
    REQUIRE_FALSE(d.update(0.0, Vec3{0.0, 0.0, 15.0}, r));  // across, but under 20
    REQUIRE(d.update(0.0, Vec3{0.0, 0.0, 25.0}, r));
}

TEST_CASE("weave detector holds for 0.5 s after the last sighting", "[speed]") {
    WeaveDetector d;
    const Vec3 r{300.0, 0.0, 0.0};
    REQUIRE(d.update(1.0, Vec3{0.0, 0.0, 50.0}, r));
    REQUIRE(d.update(1.4, Vec3::Zero(), r));
    REQUIRE(d.update(1.5, Vec3::Zero(), r));
    REQUIRE_FALSE(d.update(1.51, Vec3::Zero(), r));
}
