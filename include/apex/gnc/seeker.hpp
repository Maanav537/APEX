#pragma once

#include <cstdint>
#include <random>

#include <apex/core/types.hpp>

namespace apex {

// Seeker settings. Speeds, noise levels, and gains here are made up for the exercise.
struct SeekerParams {
    double range_sigma = 1.0;    // m, Gaussian noise on each range reading
    double angle_sigma = 1e-3;   // rad, Gaussian noise on each angle reading
    double alpha = 0.2;          // how far r_hat moves toward each reading
    double beta = 0.2 * 0.2 / (2.0 - 0.2);  // = alpha^2 / (2 - alpha) = 1/45
    double dt = 0.01;            // s, time between updates (100 Hz guidance)
    std::uint32_t seed = 12345;  // fixed: same noise every run, so tests repeat
};

// The cat's sensor plus an alpha-beta filter.
//
// Each update() takes the true LOS r (mouse - cat, world frame), turns it into
// a noisy reading of range, azimuth, and elevation, converts that back into a
// noisy r (z), then filters:
//   predict   r_pred = r_hat + v_hat * dt
//   error     e      = z - r_pred
//   correct   r_hat  = r_pred + alpha * e
//             v_hat  = v_hat + (beta / dt) * e
// The first update just sets r_hat = z and v_hat = 0.
// v_hat estimates v_rel = v_mouse - v_cat without ever differencing raw readings.
class Seeker {
public:
    explicit Seeker(const SeekerParams& params = {});

    void update(const Vec3& r_true);

    const Vec3& r_hat() const { return r_hat_; }  // m, world frame
    const Vec3& v_hat() const { return v_hat_; }  // m/s, world frame
    const Vec3& last_reading() const { return z_; }  // m, the noisy r before filtering

private:
    // Truth -> noisy range/azimuth/elevation -> noisy r.
    Vec3 measure(const Vec3& r_true);

    SeekerParams params_;
    std::mt19937 rng_;
    std::normal_distribution<double> unit_noise_{0.0, 1.0};

    Vec3 r_hat_ = Vec3::Zero();
    Vec3 v_hat_ = Vec3::Zero();
    Vec3 z_ = Vec3::Zero();
    bool started_ = false;
};

}  // namespace apex
