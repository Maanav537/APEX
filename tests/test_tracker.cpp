#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <apex/gnc/tracker.hpp>

using apex::Tracker;
using apex::TrackerParams;
using apex::Vec3;
using Catch::Matchers::WithinAbs;

namespace {

TrackerParams no_noise() {
    TrackerParams p;
    p.range_sigma = 0.0;
    p.angle_sigma = 0.0;
    return p;
}

}  // namespace

TEST_CASE("beta and gamma defaults match the pairing formulas", "[tracker]") {
    const TrackerParams p;
    REQUIRE(p.alpha == 0.05);
    REQUIRE_THAT(p.beta, WithinAbs(2.0 * (2.0 - p.alpha) - 4.0 * std::sqrt(1.0 - p.alpha), 1e-15));
    REQUIRE_THAT(p.gamma, WithinAbs(p.beta * p.beta / (2.0 * p.alpha), 1e-15));
}

TEST_CASE("first update: p_hat is own position plus the reading", "[tracker]") {
    Tracker tracker{no_noise()};
    const Vec3 own{10.0, -20.0, 1000.0};
    const Vec3 r{200.0, 300.0, -50.0};
    tracker.update(own, r);
    REQUIRE((tracker.p_hat() - (own + r)).norm() < 1e-9);
    REQUIRE(tracker.v_hat().isZero());
    REQUIRE(tracker.a_hat().isZero());
}

TEST_CASE("no noise: a_hat converges to a constant cat acceleration", "[tracker]") {
    // Cat turning at a steady 30 m/s^2 sideways; mouse sitting still.
    const TrackerParams p = no_noise();
    Tracker tracker{p};
    const Vec3 own{0.0, 0.0, 1000.0};
    const Vec3 p0{300.0, -100.0, 1000.0};
    const Vec3 v0{-60.0, 0.0, 0.0};
    const Vec3 a{0.0, 30.0, 0.0};

    for (int k = 0; k < 2000; ++k) {
        const double t = k * p.dt;
        const Vec3 cat = p0 + v0 * t + a * (0.5 * t * t);
        tracker.update(own, cat - own);
    }
    REQUIRE((tracker.a_hat() - a).norm() < 1e-3);
}

TEST_CASE("mouse's own turn does not show up in a_hat", "[tracker]") {
    // Mouse circling at 20 m/s with 50 m/s^2 of turn; cat flying straight.
    // The filter tracks the cat's absolute position, so a_hat should be ~0.
    const TrackerParams p = no_noise();
    Tracker tracker{p};
    const double speed = 20.0;
    const double omega = 50.0 / speed;  // rad/s, a = v * omega
    const double radius = speed / omega;
    const Vec3 cat0{-400.0, 200.0, 1000.0};
    const Vec3 cat_v{60.0, 0.0, 0.0};

    // 40 s: the start-up error (v_hat = 0 vs 60 m/s) dies out with a ~1.7 s
    // time constant at alpha = 0.05; 20 s still leaves ~1e-4 m/s^2.
    for (int k = 0; k < 4000; ++k) {
        const double t = k * p.dt;
        const Vec3 own{radius * std::cos(omega * t), radius * std::sin(omega * t), 1000.0};
        const Vec3 cat = cat0 + cat_v * t;
        tracker.update(own, cat - own);
    }
    REQUIRE(tracker.a_hat().norm() < 1e-6);
    REQUIRE((tracker.v_hat() - cat_v).norm() < 1e-6);
}

TEST_CASE("noise: a_hat stays small for a cat flying straight", "[tracker]") {
    // Still cat 300 m out along +x. Expected spread about 1.5 m/s^2 per metre
    // of reading noise: ~1.5 along the LOS (1 m range noise), ~0.45 across it.
    const TrackerParams p;
    Tracker tracker{p};
    const Vec3 own{0.0, 0.0, 1000.0};
    const Vec3 r{300.0, 0.0, 0.0};

    constexpr int settle = 1000;
    constexpr int samples = 5000;
    double along_sq = 0.0;
    double across_sq = 0.0;
    for (int k = 0; k < settle + samples; ++k) {
        tracker.update(own, r);
        if (k >= settle) {
            const Vec3& a = tracker.a_hat();
            along_sq += a.x() * a.x();
            across_sq += a.y() * a.y() + a.z() * a.z();
        }
    }
    const double along_rms = std::sqrt(along_sq / samples);
    const double across_rms = std::sqrt(across_sq / (2.0 * samples));  // per axis
    INFO("a_hat rms along LOS = " << along_rms << ", across (per axis) = " << across_rms);

    REQUIRE(along_rms < 3.0);
    REQUIRE(across_rms < 1.0);
}
