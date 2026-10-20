#include <apex/gnc/seeker.hpp>

#include <cmath>

namespace apex {

Seeker::Seeker(const SeekerParams& params) : params_(params), rng_(params.seed) {}

Vec3 Seeker::measure(const Vec3& r_true) {
    // Spherical reading, world frame: azimuth around z from +x, elevation up from the x-y plane.
    const double range = r_true.norm();
    const double azimuth = std::atan2(r_true.y(), r_true.x());
    const double elevation = std::atan2(r_true.z(), std::hypot(r_true.x(), r_true.y()));

    const double range_m = range + params_.range_sigma * unit_noise_(rng_);
    const double az_m = azimuth + params_.angle_sigma * unit_noise_(rng_);
    const double el_m = elevation + params_.angle_sigma * unit_noise_(rng_);

    // Back to x, y, z.
    const double flat = range_m * std::cos(el_m);
    return Vec3{flat * std::cos(az_m), flat * std::sin(az_m), range_m * std::sin(el_m)};
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
