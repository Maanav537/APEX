#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <apex/physics/atmosphere.hpp>

using apex::StdAtmosphere1976;
using Catch::Matchers::WithinRel;

// Reference values: U.S. Standard Atmosphere 1976 tables (geometric altitude).

TEST_CASE("Atmosphere matches 1976 tables at sea level", "[atmosphere]") {
    const auto s = StdAtmosphere1976{}.at(0.0);
    REQUIRE_THAT(s.density, WithinRel(1.2250, 1e-4));
    REQUIRE_THAT(s.pressure, WithinRel(101325.0, 1e-6));
    REQUIRE_THAT(s.speed_of_sound, WithinRel(340.294, 1e-4));
}

TEST_CASE("Atmosphere matches 1976 tables in upper layers", "[atmosphere]") {
    const StdAtmosphere1976 atm;

    const auto s11 = atm.at(11000.0);  // just above the tropopause (geopotential 10981 m)
    REQUIRE_THAT(s11.density, WithinRel(0.36480, 1e-3));
    REQUIRE_THAT(s11.pressure, WithinRel(22700.0, 1e-3));
    REQUIRE_THAT(s11.speed_of_sound, WithinRel(295.15, 1e-3));

    const auto s20 = atm.at(20000.0);  // isothermal layer
    REQUIRE_THAT(s20.density, WithinRel(0.088910, 1e-3));
    REQUIRE_THAT(s20.pressure, WithinRel(5529.3, 1e-3));
    REQUIRE_THAT(s20.speed_of_sound, WithinRel(295.07, 1e-3));
}

TEST_CASE("Atmosphere clamps altitudes outside the model range", "[atmosphere]") {
    const StdAtmosphere1976 atm;

    const auto below = atm.at(-20000.0);
    const auto floor = atm.at(StdAtmosphere1976::min_altitude);
    REQUIRE(below.density == floor.density);
    REQUIRE(below.pressure == floor.pressure);
    REQUIRE(below.speed_of_sound == floor.speed_of_sound);

    const auto above = atm.at(150000.0);
    const auto ceiling = atm.at(StdAtmosphere1976::max_altitude);
    REQUIRE(above.density == ceiling.density);
    REQUIRE(above.pressure == ceiling.pressure);
    REQUIRE(above.speed_of_sound == ceiling.speed_of_sound);

    // -5 km is valid data: denser than sea level.
    REQUIRE_THAT(floor.density, WithinRel(1.9311, 1e-3));
}
