#include <catch2/catch_test_macros.hpp>

#include <apex/core/types.hpp>
#include <apex/core/version.hpp>

TEST_CASE("library links and reports a version", "[smoke]") {
    REQUIRE(apex::version() == "0.1.0");
}

TEST_CASE("Eigen vector addition works through Vec3", "[smoke]") {
    const apex::Vec3 a{1.0, 2.0, 3.0};
    const apex::Vec3 b{4.0, 5.0, 6.0};
    REQUIRE(a + b == apex::Vec3{5.0, 7.0, 9.0});
}
