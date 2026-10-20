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

// LOS along +x: the detector's up axis is +z and its side axis is -y.
namespace {
const Vec3 los_x{300.0, 0.0, 0.0};
double tick(int k) { return k / 100.0; }  // 100 Hz updates, exact at whole hundredths
}  // namespace

TEST_CASE("weave detector: a steady sign triggers on the 3rd full window", "[speed]") {
    WeaveDetector d;
    // Window full at update 20 (21 samples); quiet windows at 20, 21, 22.
    for (int k = 0; k < 22; ++k) {
        REQUIRE_FALSE(d.update(tick(k), Vec3{0.0, 0.0, 0.3}, los_x));
    }
    REQUIRE(d.update(tick(22), Vec3{0.0, 0.0, 0.3}, los_x));
}

TEST_CASE("weave detector: signs flipping every update never trigger", "[speed]") {
    WeaveDetector d;
    for (int k = 0; k < 300; ++k) {
        const double s = k % 2 == 0 ? 1.0 : -1.0;
        REQUIRE_FALSE(d.update(tick(k), Vec3{0.0, s, s}, los_x));
    }
}

TEST_CASE("weave detector: a weave on the side axis counts too", "[speed]") {
    WeaveDetector d;
    bool seen = false;
    for (int k = 0; k <= 22; ++k) {
        const double s = k % 2 == 0 ? 1.0 : -1.0;
        seen = d.update(tick(k), Vec3{0.0, 0.3, s}, los_x);  // up flips, side steady
    }
    REQUIRE(seen);
}

TEST_CASE("weave detector holds for 0.5 s after the last quiet run", "[speed]") {
    // e = (0, -s, s) puts s on both the up (+z) and side (-y) axes, so both flip together.
    WeaveDetector d;
    for (int k = 0; k <= 22; ++k) {
        d.update(tick(k), Vec3{0.0, -0.3, 0.3}, los_x);
    }
    // Alternating from update 23: 1, 2, 3 flips stay quiet (last seen at 25), 4 does not.
    bool seen = false;
    for (int k = 23; k <= 75; ++k) {
        const double s = k % 2 == 0 ? 0.3 : -0.3;
        seen = d.update(tick(k), Vec3{0.0, -s, s}, los_x);
    }
    REQUIRE(seen);  // 0.75 - 0.25 = 0.5 s
    REQUIRE_FALSE(d.update(tick(76), Vec3{0.0, 0.3, 0.3}, los_x));
}
