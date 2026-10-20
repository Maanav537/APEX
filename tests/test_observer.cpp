#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <apex/gnc/observer.hpp>

using apex::CatObserver;
using apex::CatObserverParams;
using apex::Vec3;
using Catch::Matchers::WithinAbs;

namespace {

CatObserverParams no_noise() {
    CatObserverParams p;
    p.range_sigma = 0.0;
    p.angle_sigma = 0.0;
    return p;
}

}  // namespace

TEST_CASE("distance-scaled R: position gain K_0 is smaller at long range", "[observer]") {
    CatObserver near_obs;
    CatObserver far_obs;
    for (int k = 0; k < 50; ++k) {
        near_obs.update(Vec3{50.0, 0.0, 0.0});
        far_obs.update(Vec3{2000.0, 0.0, 0.0});
    }
    // At 2000 m, cross-range angle noise (2 m) dominates range noise (1 m),
    // so R(2000) > R(50) and the Kalman filter trusts each reading less.
    REQUIRE(far_obs.position_gain() < near_obs.position_gain());
}

TEST_CASE("first update: p_hat/r_hat is the reading, v_hat and a_hat are zero", "[observer]") {
    CatObserver observer{no_noise()};
    const Vec3 r{200.0, 300.0, -50.0};
    observer.update(r);
    // No noise: range/azimuth/elevation and back gives r again (up to rounding).
    REQUIRE((observer.r_hat() - r).norm() < 1e-9);
    REQUIRE(observer.v_hat().isZero());
    REQUIRE(observer.a_hat().isZero());
}

TEST_CASE("innovation: zero on the first update, then reading minus prediction", "[observer]") {
    CatObserver obs(no_noise());
    obs.update(Vec3{100.0, 0.0, 0.0});
    REQUIRE(obs.innovation() == Vec3::Zero());
    // v_hat and a_hat are still zero, so the prediction is the first position.
    obs.update(Vec3{100.0, 5.0, 0.0});
    REQUIRE_THAT((obs.innovation() - Vec3{0.0, 5.0, 0.0}).norm(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("no noise: v_hat converges to a constant v_rel", "[observer]") {
    // Mouse drifting away at a steady rate. The filter starts at v_hat = 0
    // and has to learn v_rel from how r changes.
    const CatObserverParams p = no_noise();
    CatObserver observer{p};
    const Vec3 r0{300.0, -100.0, 20.0};
    const Vec3 v_rel{-55.0, 12.0, 3.0};

    for (int k = 0; k < 300; ++k) {
        observer.update(r0 + v_rel * (k * p.dt));
    }
    REQUIRE((observer.v_hat() - v_rel).norm() < 1e-6);
    REQUIRE((observer.r_hat() - (r0 + v_rel * (299 * p.dt))).norm() < 1e-6);
}

TEST_CASE("no noise: a_hat converges on a constant acceleration", "[observer]") {
    const CatObserverParams p = no_noise();
    CatObserver observer{p};
    const Vec3 p0{300.0, 0.0, 1000.0};
    const Vec3 v0{20.0, 0.0, 0.0};
    const Vec3 a_true{0.0, 15.0, -5.0};

    for (int k = 0; k < 300; ++k) {
        const double t = k * p.dt;
        observer.update(Vec3::Zero(), p0 + v0 * t + 0.5 * a_true * t * t);
    }
    REQUIRE((observer.a_hat() - a_true).norm() < 1e-5);
}

TEST_CASE("noise: v_hat spread is far below raw differencing", "[observer]") {
    // Still evader 300 m out along +x, so true v_rel = 0 and the x part of
    // each reading carries the range noise (sigma = 1 m).
    const CatObserverParams p;
    CatObserver observer{p};
    const Vec3 r{300.0, 0.0, 0.0};

    constexpr int settle = 200;
    constexpr int samples = 5000;
    double raw_sq = 0.0;
    double filtered_sq = 0.0;
    Vec3 z_prev = Vec3::Zero();

    for (int k = 0; k < settle + samples; ++k) {
        observer.update(r);
        if (k >= settle) {
            const double raw = (observer.last_reading().x() - z_prev.x()) / p.dt;
            raw_sq += raw * raw;
            filtered_sq += observer.v_hat().x() * observer.v_hat().x();
        }
        z_prev = observer.last_reading();
    }
    const double raw_rms = std::sqrt(raw_sq / samples);            // expect ~141 m/s
    const double filtered_rms = std::sqrt(filtered_sq / samples);  // expect ~3.7 m/s
    INFO("raw differencing rms = " << raw_rms << " m/s, filtered rms = " << filtered_rms << " m/s");

    REQUIRE(raw_rms > 100.0);
    REQUIRE(filtered_rms < 6.0);
}

TEST_CASE("same seed gives the same noise", "[observer]") {
    CatObserver a;
    CatObserver b;
    for (int k = 0; k < 50; ++k) {
        a.update(Vec3{300.0, 40.0, -10.0});
        b.update(Vec3{300.0, 40.0, -10.0});
    }
    REQUIRE(a.r_hat() == b.r_hat());
    REQUIRE(a.v_hat() == b.v_hat());
}
