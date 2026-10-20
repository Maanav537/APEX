#include <cmath>
#include <numbers>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <apex/gnc/evade.hpp>

using apex::evade_command;
using apex::EvadeParams;
using apex::Vec3;
using Catch::Matchers::WithinAbs;

namespace {

// Mouse flying +x at 20 m/s, goal straight ahead.
const Vec3 mouse_v{20.0, 0.0, 0.0};
const Vec3 to_goal{500.0, 0.0, 0.0};

// Cat 100 m ahead on +x, closing at 60 m/s: t_go = 100 / 80 = 1.25 s.
const Vec3 r_close{100.0, 0.0, 0.0};
const Vec3 v_close{-80.0, 0.0, 0.0};

}  // namespace

TEST_CASE("pointed at the goal, cat far: no command", "[evade]") {
    const auto cmd = evade_command(Vec3{-1.0e5, 0.0, 0.0}, Vec3{40.0, 0.0, 0.0}, mouse_v, to_goal, 0.0);
    REQUIRE(cmd.accel.isZero());
    REQUIRE_FALSE(cmd.dodging);
}

TEST_CASE("goal push is sideways toward the goal and shrinks as the mouse lines up", "[evade]") {
    // Goal 90 degrees left: full push along +y.
    const auto side =
        evade_command(Vec3{-1.0e5, 0.0, 0.0}, Vec3::Zero(), mouse_v, Vec3{0.0, 300.0, 0.0}, 0.0);
    REQUIRE((side.accel - Vec3{0.0, 50.0, 0.0}).norm() < 1e-9);

    // Goal 30 degrees left: push = 50 * sin(30) along the sideways direction.
    const Vec3 g{std::cos(std::numbers::pi / 6), std::sin(std::numbers::pi / 6), 0.0};
    const auto off = evade_command(Vec3{-1.0e5, 0.0, 0.0}, Vec3::Zero(), mouse_v, g, 0.0);
    REQUIRE_THAT(off.accel.norm(), WithinAbs(25.0, 1e-9));
    REQUIRE_THAT(off.accel.dot(mouse_v), WithinAbs(0.0, 1e-9));
}

TEST_CASE("t_go is range over closing speed; infinite when opening", "[evade]") {
    const auto closing = evade_command(r_close, v_close, mouse_v, to_goal, 0.0);
    REQUIRE_THAT(closing.t_go, WithinAbs(1.25, 1e-12));

    const auto opening = evade_command(r_close, -v_close, mouse_v, to_goal, 0.0);
    REQUIRE(std::isinf(opening.t_go));
    REQUIRE_FALSE(opening.dodging);
}

TEST_CASE("weave: first leg is full-size, up, and across the LOS", "[evade]") {
    const auto cmd = evade_command(r_close, v_close, mouse_v, to_goal, 0.0);
    REQUIRE(cmd.dodging);
    REQUIRE((cmd.accel - Vec3{0.0, 0.0, 50.0}).norm() < 1e-9);

    // Cat ahead and above: "up" tilts so the weave stays across the LOS.
    const Vec3 r_up{100.0, 0.0, 100.0};
    const auto tilted = evade_command(r_up, Vec3{-80.0, 0.0, -80.0}, mouse_v, to_goal, 0.0);
    REQUIRE(tilted.dodging);
    REQUIRE_THAT(tilted.accel.dot(r_up), WithinAbs(0.0, 1e-9));
    REQUIRE(tilted.accel.z() > 0.0);
}

TEST_CASE("weave flips direction after each leg", "[evade]") {
    const EvadeParams p;
    const double mid1 = 0.5 * p.weave_leg;  // first leg
    const double mid2 = 1.5 * p.weave_leg;  // second leg
    const double mid3 = 2.5 * p.weave_leg;  // third leg
    REQUIRE(evade_command(r_close, v_close, mouse_v, to_goal, mid1).accel.z() > 0.0);
    REQUIRE(evade_command(r_close, v_close, mouse_v, to_goal, mid2).accel.z() < 0.0);
    REQUIRE(evade_command(r_close, v_close, mouse_v, to_goal, mid3).accel.z() > 0.0);
}

TEST_CASE("cat straight above: weave falls back to a horizontal direction", "[evade]") {
    const auto cmd = evade_command(Vec3{0.0, 0.0, 100.0}, Vec3{0.0, 0.0, -80.0}, mouse_v, to_goal, 0.0);
    REQUIRE(cmd.dodging);
    REQUIRE(std::isfinite(cmd.accel.norm()));
    REQUIRE_THAT(cmd.accel.z(), WithinAbs(0.0, 1e-9));
}

TEST_CASE("no weave while t_go is at or above the threshold", "[evade]") {
    // 240 m at 80 m/s closing: t_go = 3 s exactly.
    const auto cmd = evade_command(Vec3{240.0, 0.0, 0.0}, v_close, mouse_v, to_goal, 0.0);
    REQUIRE_THAT(cmd.t_go, WithinAbs(3.0, 1e-12));
    REQUIRE_FALSE(cmd.dodging);
}

TEST_CASE("goal push and weave are summed, not limited", "[evade]") {
    // Goal left (+50 y) and weave up (+50 z): the caller limits it, with the speed push.
    const auto cmd = evade_command(r_close, v_close, mouse_v, Vec3{0.0, 300.0, 0.0}, 0.0);
    REQUIRE(cmd.dodging);
    REQUIRE((cmd.accel - Vec3{0.0, 50.0, 50.0}).norm() < 1e-9);
}
