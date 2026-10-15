#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <atomic>
#include <cstdlib>
#include <new>

#include <apex/sim/simulation.hpp>

using apex::PointMassState;
using apex::Simulation;
using apex::Vec3;
using Catch::Matchers::WithinAbs;

// Count every heap allocation in this test program. Replacing the global
// operator new is the standard way to observe allocations.
namespace {
std::atomic<std::size_t> g_allocations{0};
}

void* operator new(std::size_t size) {
    ++g_allocations;
    if (void* p = std::malloc(size == 0 ? 1 : size)) {
        return p;
    }
    throw std::bad_alloc{};
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

namespace {

// Horizontal circle, r = 100 m, at 1000 m altitude.
Simulation make_circle() {
    const double r = 100.0;
    const double omega = 0.5;
    const Vec3 center{0.0, 0.0, 1000.0};
    return Simulation{{center + Vec3{r, 0.0, 0.0}, Vec3{0.0, r * omega, 0.0}}, center, omega};
}

}  // namespace

TEST_CASE("allocation counter sees heap allocations", "[sim]") {
    // Guards the zero-allocation test: if the counter never counts, that test means nothing.
    const std::size_t before = g_allocations.load();
    auto* p = new int(42);
    delete p;
    REQUIRE(g_allocations.load() - before == 1);
}

TEST_CASE("step() makes zero heap allocations", "[sim]") {
    Simulation sim = make_circle();

    const std::size_t before = g_allocations.load();
    for (int i = 0; i < 1000; ++i) {
        sim.step();
    }
    const std::size_t after = g_allocations.load();

    REQUIRE(after - before == 0);
}

TEST_CASE("step() advances the clock by dt each tick", "[sim]") {
    Simulation sim = make_circle();
    for (int i = 0; i < 1000; ++i) {
        sim.step();
    }
    REQUIRE_THAT(sim.time(), WithinAbs(1.0, 1e-9));
    REQUIRE_FALSE(sim.ground_hit());
}

TEST_CASE("step() flags a ground hit and then stops", "[sim]") {
    // Vertical circle centered 50 m up with r = 100 m: it dips below z = 0.
    const double omega = 0.5;
    const Vec3 center{0.0, 0.0, 50.0};
    Simulation sim{{center + Vec3{100.0, 0.0, 0.0}, Vec3{0.0, 0.0, -50.0}}, center, omega};

    int ticks = 0;
    while (!sim.ground_hit() && ticks < 20000) {
        sim.step();
        ++ticks;
    }
    REQUIRE(sim.ground_hit());
    REQUIRE(sim.mouse().position.z() <= 0.0);

    const PointMassState frozen = sim.mouse();
    const double frozen_t = sim.time();
    sim.step();
    REQUIRE(sim.mouse().position == frozen.position);
    REQUIRE(sim.time() == frozen_t);
}
