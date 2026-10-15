#include <catch2/catch_test_macros.hpp>

#include <apex/core/types.hpp>

using apex::PointMassState;
using apex::Vec3;

TEST_CASE("PointMassState defaults to zero", "[types]") {
    const PointMassState s;
    REQUIRE(s.position == Vec3::Zero());
    REQUIRE(s.velocity == Vec3::Zero());
}

TEST_CASE("PointMassState adds field by field", "[types]") {
    const PointMassState a{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const PointMassState b{{10.0, 20.0, 30.0}, {40.0, 50.0, 60.0}};
    const PointMassState c = a + b;
    REQUIRE(c.position == Vec3{11.0, 22.0, 33.0});
    REQUIRE(c.velocity == Vec3{44.0, 55.0, 66.0});
}

TEST_CASE("PointMassState scales by a scalar from either side", "[types]") {
    const PointMassState x{{1.0, -2.0, 3.0}, {0.5, 0.0, -1.0}};
    const PointMassState left = 2.0 * x;
    const PointMassState right = x * 2.0;
    REQUIRE(left.position == Vec3{2.0, -4.0, 6.0});
    REQUIRE(left.velocity == Vec3{1.0, 0.0, -2.0});
    REQUIRE(right.position == left.position);
    REQUIRE(right.velocity == left.velocity);
}

TEST_CASE("PointMassState supports the RK4 form x + dt * k", "[types]") {
    const PointMassState x{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    const PointMassState k{{1.0, 0.0, 0.0}, {0.0, 2.0, 0.0}};  // derivative: velocity, acceleration
    const PointMassState next = x + 0.5 * k;
    REQUIRE(next.position == Vec3{0.5, 0.0, 0.0});
    REQUIRE(next.velocity == Vec3{1.0, 1.0, 0.0});
}
