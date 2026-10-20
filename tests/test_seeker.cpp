#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <apex/gnc/seeker.hpp>

using apex::Seeker;
using apex::SeekerParams;
using apex::Vec3;
using Catch::Matchers::WithinAbs;

namespace {

SeekerParams no_noise() {
    SeekerParams p;
    p.range_sigma = 0.0;
    p.angle_sigma = 0.0;
    return p;
}

}  // namespace

TEST_CASE("beta default matches the pairing formula", "[seeker]") {
    const SeekerParams p;
    REQUIRE_THAT(p.beta, WithinAbs(p.alpha * p.alpha / (2.0 - p.alpha), 1e-15));
    REQUIRE_THAT(p.beta, WithinAbs(1.0 / 45.0, 1e-15));
}

TEST_CASE("first update: r_hat is the reading, v_hat is zero", "[seeker]") {
    Seeker seeker{no_noise()};
    const Vec3 r{200.0, 300.0, -50.0};
    seeker.update(r);
    // No noise: range/azimuth/elevation and back gives r again (up to rounding).
    REQUIRE((seeker.r_hat() - r).norm() < 1e-9);
    REQUIRE(seeker.v_hat().isZero());
}

TEST_CASE("no noise: v_hat converges to a constant v_rel", "[seeker]") {
    // Mouse drifting away at a steady rate. The filter starts at v_hat = 0
    // and has to learn v_rel from how r changes.
    const SeekerParams p = no_noise();
    Seeker seeker{p};
    const Vec3 r0{300.0, -100.0, 20.0};
    const Vec3 v_rel{-55.0, 12.0, 3.0};

    for (int k = 0; k < 300; ++k) {
        seeker.update(r0 + v_rel * (k * p.dt));
    }
    REQUIRE((seeker.v_hat() - v_rel).norm() < 1e-6);
    REQUIRE((seeker.r_hat() - (r0 + v_rel * (299 * p.dt))).norm() < 1e-6);
}

TEST_CASE("noise: v_hat spread is far below raw differencing", "[seeker]") {
    // Still target 300 m out along +x, so true v_rel = 0 and the x part of
    // each reading carries the range noise (sigma = 1 m).
    const SeekerParams p;
    Seeker seeker{p};
    const Vec3 r{300.0, 0.0, 0.0};

    constexpr int settle = 200;
    constexpr int samples = 5000;
    double raw_sq = 0.0;
    double filtered_sq = 0.0;
    Vec3 z_prev = Vec3::Zero();

    for (int k = 0; k < settle + samples; ++k) {
        seeker.update(r);
        if (k >= settle) {
            const double raw = (seeker.last_reading().x() - z_prev.x()) / p.dt;
            raw_sq += raw * raw;
            filtered_sq += seeker.v_hat().x() * seeker.v_hat().x();
        }
        z_prev = seeker.last_reading();
    }
    const double raw_rms = std::sqrt(raw_sq / samples);            // expect ~141 m/s
    const double filtered_rms = std::sqrt(filtered_sq / samples);  // expect ~3.7 m/s
    INFO("raw differencing rms = " << raw_rms << " m/s, filtered rms = " << filtered_rms << " m/s");

    REQUIRE(raw_rms > 100.0);
    REQUIRE(filtered_rms < 6.0);
}

TEST_CASE("same seed gives the same noise", "[seeker]") {
    Seeker a;
    Seeker b;
    for (int k = 0; k < 50; ++k) {
        a.update(Vec3{300.0, 40.0, -10.0});
        b.update(Vec3{300.0, 40.0, -10.0});
    }
    REQUIRE(a.r_hat() == b.r_hat());
    REQUIRE(a.v_hat() == b.v_hat());
}
