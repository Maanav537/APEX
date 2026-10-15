#pragma once

namespace apex {

// Air properties at one altitude. SI units.
struct AtmosphereSample {
    double density = 0.0;         // kg/m^3
    double pressure = 0.0;        // Pa
    double speed_of_sound = 0.0;  // m/s
};

// 1976 US Standard Atmosphere, 0-86 km layers (extended down to -5 km).
//
// Altitudes outside [min_altitude, max_altitude] are clamped to the nearest
// limit. Ground contact is not this model's job; the sim loop checks that.
// Models are swapped at compile time: code that needs air takes the model as a
// template parameter, like `f` in rk4_step.
class StdAtmosphere1976 {
public:
    static constexpr double min_altitude = -5000.0;  // m, geometric
    static constexpr double max_altitude = 86000.0;  // m, geometric

    // `h` is geometric altitude above mean sea level, in meters.
    AtmosphereSample at(double h) const;
};

}  // namespace apex
