#pragma once

#include <cmath>
#include <cstdint>
#include <random>

#include <apex/core/types.hpp>

namespace apex {

// Gain pairing for an alpha-beta-gamma filter: beta and gamma follow from alpha.
inline double abg_beta(double alpha) { return 2.0 * (2.0 - alpha) - 4.0 * std::sqrt(1.0 - alpha); }
inline double abg_gamma(double alpha) {
    const double beta = abg_beta(alpha);
    return beta * beta / (2.0 * alpha);
}

// Tracker settings. Noise levels and gains here are made up for the exercise.
struct TrackerParams {
    double range_sigma = 1.0;      // m, Gaussian noise on each range reading
    double angle_sigma = 1e-3;     // rad, Gaussian noise on each angle reading
    double alpha = 0.05;           // how far p_hat moves toward each reading
    double beta = abg_beta(0.05);  // velocity share of the error
    double gamma = abg_gamma(0.05);  // acceleration share of the error
    double dt = 0.01;              // s, time between updates (100 Hz)
    std::uint32_t seed = 54321;    // fixed, and not the observer's seed
};

// The mouse's sensor plus an alpha-beta-gamma filter on the cat's absolute
// position.
//
// Each update() takes the mouse's own position and the true LOS (cat - mouse),
// makes a noisy bearing/range sensor reading of the LOS, and adds the mouse's own position:
// z = p_mouse + reading is a noisy cat position. The mouse's own motion is
// gone before filtering, so a_hat is the cat's acceleration alone.
//   predict   p_pred = p_hat + v_hat * dt + a_hat * dt^2 / 2
//             v_pred = v_hat + a_hat * dt
//   error     e      = z - p_pred
//   correct   p_hat  = p_pred + alpha * e
//             v_hat  = v_pred + (beta / dt) * e
//             a_hat  = a_hat  + (2 * gamma / dt^2) * e
// The first update just sets p_hat = z, v_hat = a_hat = 0.
class Tracker {
public:
    explicit Tracker(const TrackerParams& params = {});

    void update(const Vec3& own_position, const Vec3& r_true);

    const Vec3& p_hat() const { return p_hat_; }  // m, world frame
    const Vec3& v_hat() const { return v_hat_; }  // m/s, world frame
    const Vec3& a_hat() const { return a_hat_; }  // m/s^2, world frame
    const Vec3& last_reading() const { return z_; }  // m, noisy cat position before filtering

private:
    TrackerParams params_;
    std::mt19937 rng_;
    std::normal_distribution<double> unit_noise_{0.0, 1.0};

    Vec3 p_hat_ = Vec3::Zero();
    Vec3 v_hat_ = Vec3::Zero();
    Vec3 a_hat_ = Vec3::Zero();
    Vec3 z_ = Vec3::Zero();
    bool started_ = false;
};

}  // namespace apex
