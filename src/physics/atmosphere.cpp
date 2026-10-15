#include <apex/physics/atmosphere.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace apex {
namespace {

constexpr double g0 = 9.80665;       // m/s^2, standard gravity
constexpr double R = 287.0531;       // J/(kg K), R* / M0 for dry air
constexpr double gamma_air = 1.4;    // ratio of specific heats
constexpr double r_earth = 6356766.0;  // m, radius used by the 1976 standard

// One layer: temperature varies linearly with geopotential altitude H.
struct Layer {
    double base_H;         // m, geopotential
    double base_T;         // K
    double lapse;          // K/m
    double base_pressure;  // Pa
};

// Fixed-size table: no heap use when the model is queried.
constexpr std::array<Layer, 7> layers{{
    {0.0, 288.15, -0.0065, 101325.0},
    {11000.0, 216.65, 0.0, 22632.06},
    {20000.0, 216.65, 0.001, 5474.889},
    {32000.0, 228.65, 0.0028, 868.0187},
    {47000.0, 270.65, 0.0, 110.9063},
    {51000.0, 270.65, -0.0028, 66.93887},
    {71000.0, 214.65, -0.002, 3.956420},
}};

}  // namespace

AtmosphereSample StdAtmosphere1976::at(double h) const {
    h = std::clamp(h, min_altitude, max_altitude);

    // Layers are defined in geopotential altitude, which accounts for gravity
    // weakening with height.
    const double H = r_earth * h / (r_earth + h);

    // Highest layer whose base is at or below H. Below 0 m, layer 0 extends down.
    std::size_t i = 0;
    while (i + 1 < layers.size() && H >= layers[i + 1].base_H) {
        ++i;
    }
    const Layer& L = layers[i];

    const double dH = H - L.base_H;
    const double T = L.base_T + L.lapse * dH;

    // Hydrostatic pressure: power law when T changes, exponential when constant.
    const double p = (L.lapse != 0.0)
                         ? L.base_pressure * std::pow(L.base_T / T, g0 / (R * L.lapse))
                         : L.base_pressure * std::exp(-g0 * dH / (R * L.base_T));

    return {p / (R * T), p, std::sqrt(gamma_air * R * T)};
}

}  // namespace apex
