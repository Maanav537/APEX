#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <apex/gnc/pn.hpp>

using apex::limit_accel;
using apex::pn_command;
using apex::Vec3;
using Catch::Matchers::WithinAbs;

TEST_CASE("PN commands nothing on a collision course", "[pn]") {
    // Cat 100 m behind, flying straight at a still mouse: the LOS doesn't swing.
    const Vec3 a =
        pn_command(Vec3{100.0, 0.0, 0.0}, Vec3{-60.0, 0.0, 0.0}, Vec3{60.0, 0.0, 0.0}, 3.0);
    REQUIRE(a.norm() < 1e-12);
}

TEST_CASE("PN pushes toward the way the LOS swings", "[pn]") {
    // Cat flies +x at 60, mouse drifts +y at 10. W = (0, 0, 0.1),
    // W x v_cat = (0, 6, 0): a = 3 * 6 along +y. (True PN agrees here: v is along r.)
    const Vec3 a =
        pn_command(Vec3{100.0, 0.0, 0.0}, Vec3{-60.0, 10.0, 0.0}, Vec3{60.0, 0.0, 0.0}, 3.0);
    REQUIRE_THAT(a.x(), WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(a.y(), WithinAbs(18.0, 1e-12));
    REQUIRE_THAT(a.z(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("PN command is sideways to the cat's own velocity", "[pn]") {
    // Only turns the cat; no part of it speeds the cat up or slows it down.
    const Vec3 v_cat{60.0, 0.0, 0.0};
    const Vec3 a = pn_command(Vec3{100.0, 0.0, 0.0}, Vec3{-60.0, 10.0, 5.0}, v_cat, 3.0);
    REQUIRE(a.norm() > 0.0);
    REQUIRE_THAT(a.dot(v_cat.normalized()), WithinAbs(0.0, 1e-12));
}

TEST_CASE("PN command need not be sideways to the LOS", "[pn]") {
    // Cat flying off the LOS: a stays sideways to v_cat, which tilts it
    // off sideways-to-LOS (a . r_hat is about 13.7 m/s^2 here).
    const Vec3 r{200.0, 300.0, -50.0};
    const Vec3 v_cat{50.0, -20.0, 30.0};
    const Vec3 a = pn_command(r, Vec3{-40.0, 7.0, 12.0}, v_cat, 3.0);
    REQUIRE_THAT(a.dot(v_cat.normalized()), WithinAbs(0.0, 1e-12));
    REQUIRE(std::abs(a.dot(r.normalized())) > 1.0);
}

TEST_CASE("PN returns zero when the gap is zero", "[pn]") {
    const Vec3 a = pn_command(Vec3::Zero(), Vec3{-60.0, 10.0, 0.0}, Vec3{60.0, 0.0, 0.0}, 3.0);
    REQUIRE(a.allFinite());
    REQUIRE(a.isZero());
}

TEST_CASE("limit_accel scales the whole vector, keeping direction", "[pn]") {
    // The math-board example: |(300, 0, 400)| = 500, scale by 100 / 500.
    const Vec3 a = limit_accel(Vec3{300.0, 0.0, 400.0}, 100.0);
    REQUIRE_THAT(a.x(), WithinAbs(60.0, 1e-12));
    REQUIRE_THAT(a.y(), WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(a.z(), WithinAbs(80.0, 1e-12));
}

TEST_CASE("limit_accel leaves commands under the cap alone", "[pn]") {
    const Vec3 a{30.0, -40.0, 0.0};
    REQUIRE(limit_accel(a, 100.0) == a);
}
