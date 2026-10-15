#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

#include <apex/core/types.hpp>
#include <apex/physics/rigid_body.hpp>
#include <apex/physics/rk4.hpp>

using apex::Mat3;
using apex::Quat;
using apex::RigidBodyState;
using apex::Vec3;
using apex::Wrench;

namespace {

Mat3 diag(double a, double b, double c) {
    return Vec3{a, b, c}.asDiagonal();
}

}  // namespace

TEST_CASE("Constant force gives v-dot = F / m", "[rigid_body]") {
    const auto params = apex::make_rigid_body_params(2.0, diag(1.0, 1.0, 1.0));
    const auto forces = [](double, const RigidBodyState&) {
        return Wrench{{4.0, 0.0, -6.0}, Vec3::Zero()};
    };
    RigidBodyState x;
    x.velocity = {1.0, 2.0, 3.0};
    const RigidBodyState dx = apex::rigid_body_derivative(0.0, x, params, forces);
    REQUIRE(dx.position == x.velocity);
    REQUIRE(dx.velocity == Vec3{2.0, 0.0, -3.0});
    REQUIRE(dx.angular_velocity == Vec3::Zero());
}

TEST_CASE("Constant torque on a body at rest gives w-dot = I^-1 tau", "[rigid_body]") {
    const auto params = apex::make_rigid_body_params(1.0, diag(2.0, 4.0, 5.0));
    const auto forces = [](double, const RigidBodyState&) {
        return Wrench{Vec3::Zero(), {2.0, 2.0, 10.0}};
    };
    const RigidBodyState dx = apex::rigid_body_derivative(0.0, RigidBodyState{}, params, forces);
    REQUIRE(dx.angular_velocity.isApprox(Vec3{1.0, 0.5, 2.0}));
}

TEST_CASE("Torque-free spin about a principal axis keeps w constant", "[rigid_body]") {
    const auto params = apex::make_rigid_body_params(1.0, diag(2.0, 3.0, 4.0));
    const auto forces = [](double, const RigidBodyState&) { return Wrench{}; };
    RigidBodyState x;
    x.angular_velocity = {0.0, 0.0, 5.0};
    const RigidBodyState dx = apex::rigid_body_derivative(0.0, x, params, forces);
    REQUIRE(dx.angular_velocity.isZero());  // gyroscopic term vanishes
}

TEST_CASE("Spin about z for 1 s turns q by the expected angle", "[rigid_body]") {
    const auto params = apex::make_rigid_body_params(1.0, diag(2.0, 3.0, 4.0));
    const auto forces = [](double, const RigidBodyState&) { return Wrench{}; };
    const auto f = [&](double t, const RigidBodyState& s) {
        return apex::rigid_body_derivative(t, s, params, forces);
    };

    const double half_pi = std::numbers::pi / 2.0;
    RigidBodyState x;
    x.angular_velocity = {0.0, 0.0, half_pi};  // 90 deg/s
    const double dt = 0.001;
    double t = 0.0;
    for (int i = 0; i < 1000; ++i) {
        x = apex::rk4_step(f, x, t, dt);
        x.orientation.normalize();  // what Simulation::step() will do
        t += dt;
    }

    const Quat expected(std::cos(half_pi / 2.0), 0.0, 0.0, std::sin(half_pi / 2.0));
    REQUIRE(std::abs(x.orientation.norm() - 1.0) < 1e-12);
    REQUIRE(x.orientation.coeffs().isApprox(expected.coeffs(), 1e-9));

    // The rotated body x-axis should now point along world y.
    REQUIRE((x.orientation * Vec3::UnitX()).isApprox(Vec3::UnitY(), 1e-9));
}
