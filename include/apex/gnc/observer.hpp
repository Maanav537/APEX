#pragma once

#include <cstdint>
#include <random>

#include <apex/core/types.hpp>

namespace apex {

// Cat observer settings. Speeds, noise levels, and covariances here are made up for the exercise.
struct CatObserverParams {
    double range_sigma = 1.0;          // m, Gaussian noise on each range reading
    double angle_sigma = 1e-3;         // rad, Gaussian noise on each angle reading
    double accel_process_sigma = 2.0;  // m/s^2, per-step acceleration process noise for Q
    double dt = 0.01;                  // s, time between updates (100 Hz guidance)
    std::uint32_t seed = 12345;        // fixed: same noise every run, so tests repeat
};

using ObserverParams = CatObserverParams;

// The cat's sensor plus a 3-state linear Kalman filter on the mouse's absolute
// position, velocity, and acceleration.
//
// Each update() takes the cat's own position (and velocity) and the true LOS r
// (mouse - cat, world frame), makes a noisy range/azimuth/elevation reading of r,
// and adds the cat's position so z = p_cat + reading is a noisy mouse world
// position. That strips out the cat's own turns before filtering, so a_hat is
// the mouse's acceleration alone.
//
// Measurement variance scales with measured distance d = |reading|:
//   R(d) = range_sigma^2 + (d * angle_sigma)^2
// One 3x3 covariance matrix P is shared across x, y, and z:
//   predict   x_pred = F * x_hat,  P_pred = F * P * F^T + Q
//   gain      S = P_pred(0,0) + R(d),  K = P_pred.col(0) / S
//   correct   x_hat = x_pred + K * (z - p_pred),  P = (I - K * H) * P_pred
class CatObserver {
public:
    explicit CatObserver(const CatObserverParams& params = {});

    void update(const Vec3& own_position, const Vec3& r_true,
                const Vec3& own_velocity = Vec3::Zero());
    void update(const Vec3& r_true) { update(Vec3::Zero(), r_true, Vec3::Zero()); }

    const Vec3& p_hat() const { return p_hat_; }  // m, mouse world position estimate
    Vec3 r_hat() const { return p_hat_ - own_position_; }  // m, relative LOS estimate
    const Vec3& v_hat() const { return v_hat_; }  // m/s, mouse world velocity estimate
    const Vec3& a_hat() const { return a_hat_; }  // m/s^2, mouse world acceleration estimate
    const Vec3& last_reading() const { return reading_; }  // m, noisy LOS reading before filtering
    const Mat3& covariance() const { return P_; }          // 3x3 state covariance P
    double position_gain() const { return k0_; }           // latest K_0 position gain

private:
    // Truth -> noisy range/azimuth/elevation -> noisy r.
    Vec3 measure(const Vec3& r_true);

    CatObserverParams params_;
    std::mt19937 rng_;
    std::normal_distribution<double> unit_noise_{0.0, 1.0};

    Vec3 own_position_ = Vec3::Zero();
    Vec3 p_hat_ = Vec3::Zero();
    Vec3 v_hat_ = Vec3::Zero();
    Vec3 a_hat_ = Vec3::Zero();
    Vec3 reading_ = Vec3::Zero();
    Mat3 P_ = Mat3::Zero();
    double k0_ = 0.0;
    bool started_ = false;
};

}  // namespace apex
