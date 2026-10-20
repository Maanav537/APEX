#pragma once

#include <cmath>
#include <random>

#include <apex/core/types.hpp>

namespace apex {

// One noisy radar reading of `r_true` (target - self, world frame), shared by
// the cat's seeker and the mouse's tracker.
//
// Truth -> range, azimuth, elevation -> add Gaussian noise -> back to x, y, z.
// Azimuth is around z from +x; elevation is up from the x-y plane.
inline Vec3 radar_reading(const Vec3& r_true, double range_sigma, double angle_sigma,
                          std::mt19937& rng, std::normal_distribution<double>& unit_noise) {
    const double range = r_true.norm();
    const double azimuth = std::atan2(r_true.y(), r_true.x());
    const double elevation = std::atan2(r_true.z(), std::hypot(r_true.x(), r_true.y()));

    const double range_m = range + range_sigma * unit_noise(rng);
    const double az_m = azimuth + angle_sigma * unit_noise(rng);
    const double el_m = elevation + angle_sigma * unit_noise(rng);

    const double flat = range_m * std::cos(el_m);
    return Vec3{flat * std::cos(az_m), flat * std::sin(az_m), range_m * std::sin(el_m)};
}

}  // namespace apex
