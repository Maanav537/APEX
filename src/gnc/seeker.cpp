#include <apex/gnc/seeker.hpp>

#include <apex/gnc/radar.hpp>

namespace apex {

Seeker::Seeker(const SeekerParams& params) : params_(params), rng_(params.seed) {}

Vec3 Seeker::measure(const Vec3& r_true) {
    return radar_reading(r_true, params_.range_sigma, params_.angle_sigma, rng_, unit_noise_);
}

void Seeker::update(const Vec3& r_true) {
    z_ = measure(r_true);

    if (!started_) {
        r_hat_ = z_;
        v_hat_ = Vec3::Zero();
        started_ = true;
        return;
    }

    const Vec3 r_pred = r_hat_ + v_hat_ * params_.dt;
    const Vec3 e = z_ - r_pred;
    r_hat_ = r_pred + params_.alpha * e;
    v_hat_ = v_hat_ + (params_.beta / params_.dt) * e;
}

}  // namespace apex
