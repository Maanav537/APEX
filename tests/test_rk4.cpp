#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

#include <apex/core/types.hpp>
#include <apex/physics/rk4.hpp>

using apex::PointMassState;
using apex::Vec3;

TEST_CASE("RK4 keeps a 3D circular orbit within 1e-6 after one period", "[rk4]") {
    const double r = 100.0;
    const double omega = 0.5;  // rad/s
    const double dt = 0.001;   // 1000 Hz

    // Circle tilted out of the xy-plane so all three axes are exercised.
    const Vec3 u = Vec3{1.0, 0.0, 0.0};
    const Vec3 w = Vec3{0.0, 1.0, 1.0}.normalized();

    // Uniform circular motion: a = -omega^2 * position.
    auto f = [omega](double, const PointMassState& x) {
        return PointMassState{x.velocity, -omega * omega * x.position};
    };

    PointMassState x{r * u, r * omega * w};
    double t = 0.0;
    const double period = 2.0 * std::numbers::pi / omega;
    const int steps = static_cast<int>(std::lround(period / dt));

    for (int i = 0; i < steps; ++i) {
        x = apex::rk4_step(f, x, t, dt);
        t += dt;
    }

    const Vec3 expected = r * (std::cos(omega * t) * u + std::sin(omega * t) * w);
    const double drift = (x.position - expected).norm();
    INFO("position drift = " << drift);
    REQUIRE(drift < 1e-6);
}

TEST_CASE("RK4 does not modify the input state", "[rk4]") {
    const PointMassState x0{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    auto f = [](double, const PointMassState& x) {
        return PointMassState{x.velocity, Vec3::Zero()};
    };
    const PointMassState x1 = apex::rk4_step(f, x0, 0.0, 0.1);
    REQUIRE(x0.position == Vec3{1.0, 2.0, 3.0});
    REQUIRE(x1.position.isApprox(Vec3{1.4, 2.5, 3.6}));
}
