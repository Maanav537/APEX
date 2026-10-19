#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <apex/gnc/pn.hpp>

using apex::limit_accel;
using apex::pn_command;
using apex::Vec3;
using Catch::Matchers::WithinAbs;

TEST_CASE("PN commands nothing on a collision course", "[pn]") {
    // Cat 100 m behind, flying straight at a still mouse: the LOS doesn't swing.
    const Vec3 a = pn_command(Vec3{100.0, 0.0, 0.0}, Vec3{-60.0, 0.0, 0.0}, 3.0);
    REQUIRE(a.norm() < 1e-12);
}

TEST_CASE("PN pushes toward the way the LOS swings", "[pn]") {
    // Vc = 60, W = (0, 0, 0.1), W x r_hat = (0, 0.1, 0): a = 3 * 60 * 0.1 along +y.
    const Vec3 a = pn_command(Vec3{100.0, 0.0, 0.0}, Vec3{-60.0, 10.0, 0.0}, 3.0);
    REQUIRE_THAT(a.x(), WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(a.y(), WithinAbs(18.0, 1e-12));
    REQUIRE_THAT(a.z(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("PN command is sideways to the LOS", "[pn]") {
    const Vec3 r{200.0, 300.0, -50.0};
    const Vec3 a = pn_command(r, Vec3{-40.0, 7.0, 12.0}, 3.0);
    REQUIRE(a.norm() > 0.0);
    REQUIRE_THAT(a.dot(r.normalized()), WithinAbs(0.0, 1e-12));
}

TEST_CASE("PN returns zero when the gap is zero", "[pn]") {
    const Vec3 a = pn_command(Vec3::Zero(), Vec3{-60.0, 10.0, 0.0}, 3.0);
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
