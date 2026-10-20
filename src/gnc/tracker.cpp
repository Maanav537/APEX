#include <apex/gnc/tracker.hpp>

#include <apex/gnc/radar.hpp>

namespace apex {

Tracker::Tracker(const TrackerParams& params) : params_(params), rng_(params.seed) {}

void Tracker::update(const Vec3& own_position, const Vec3& r_true) {
    z_ = own_position +
         radar_reading(r_true, params_.range_sigma, params_.angle_sigma, rng_, unit_noise_);

    if (!started_) {
        p_hat_ = z_;
        v_hat_ = Vec3::Zero();
        a_hat_ = Vec3::Zero();
        started_ = true;
        return;
    }

    const double dt = params_.dt;
    const Vec3 p_pred = p_hat_ + v_hat_ * dt + a_hat_ * (0.5 * dt * dt);
    const Vec3 v_pred = v_hat_ + a_hat_ * dt;
    const Vec3 e = z_ - p_pred;
    p_hat_ = p_pred + params_.alpha * e;
    v_hat_ = v_pred + (params_.beta / dt) * e;
    a_hat_ = a_hat_ + (2.0 * params_.gamma / (dt * dt)) * e;
}

}  // namespace apex
