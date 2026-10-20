#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <new>

#include <apex/sim/capture.hpp>
#include <apex/sim/simulation.hpp>

using apex::Mat3;
using apex::RigidBodyState;
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

constexpr double mouse_speed = 20.0;  // m/s
constexpr double cat_speed = 60.0;    // m/s
const Vec3 run_start{0.0, 0.0, 1000.0};
const Vec3 run_goal{500.0, 0.0, 1000.0};

RigidBodyState make_cat(const Vec3& position) {
    RigidBodyState cat;
    cat.position = position;
    cat.velocity = Vec3{cat_speed, 0.0, 0.0};  // flying +x
    return cat;
}

// Level flight along world +x, 500 m to the goal at 20 m/s. The cat starts
// 100 km behind on the same line: it chases but can't close in time to matter.
Simulation make_run(const Vec3& from = run_start, const Vec3& to = run_goal,
                    const Vec3& cat_from = Vec3{-1.0e5, 0.0, 1000.0}) {
    RigidBodyState mouse;
    mouse.position = from;
    const auto params = apex::make_rigid_body_params(1.0, Mat3::Identity());
    return Simulation{mouse, params, to, mouse_speed, make_cat(cat_from), params, cat_speed};
}

// The demo chase: cat off to the side and behind, so PN has to turn it.
Simulation make_chase() { return make_run(run_start, run_goal, Vec3{-200.0, -300.0, 1000.0}); }

}  // namespace

TEST_CASE("allocation counter sees heap allocations", "[sim]") {
    // Guards the zero-allocation test: if the counter never counts, that test means nothing.
    const std::size_t before = g_allocations.load();
    // volatile pointer: the compiler can't prove the int is unused, so it can't
    // elide the new/delete pair (GCC does at -O2 otherwise).
    int* volatile p = new int(42);
    delete p;
    REQUIRE(g_allocations.load() - before == 1);
}

TEST_CASE("step() makes zero heap allocations", "[sim]") {
    Simulation sim = make_run();

    const std::size_t before = g_allocations.load();
    for (int i = 0; i < 1000; ++i) {
        sim.step();
    }
    const std::size_t after = g_allocations.load();

    REQUIRE(after - before == 0);
}

TEST_CASE("step() advances the clock by dt each tick", "[sim]") {
    Simulation sim = make_run();
    for (int i = 0; i < 1000; ++i) {
        sim.step();
    }
    REQUIRE_THAT(sim.time(), WithinAbs(1.0, 1e-9));
    REQUIRE_FALSE(sim.ground_hit());
    REQUIRE_FALSE(sim.reached_goal());
}

TEST_CASE("mouse stays on the start-goal line at constant speed", "[sim]") {
    Simulation sim = make_run();
    for (int i = 0; i < 20000; ++i) {
        sim.step();
        const RigidBodyState& m = sim.mouse();
        REQUIRE(std::abs(m.position.y()) < 1e-9);
        REQUIRE(std::abs(m.position.z() - 1000.0) < 1e-9);
        REQUIRE_THAT(m.velocity.norm(), WithinAbs(mouse_speed, 1e-9));
    }
}

TEST_CASE("nose points along the velocity", "[sim]") {
    // Diagonal climb so the nose has to turn off body +x on more than one axis.
    Simulation sim = make_run(run_start, Vec3{300.0, 200.0, 1100.0});
    for (int i = 0; i < 1000; ++i) {
        sim.step();
        const Vec3 nose = sim.mouse().orientation * Vec3::UnitX();
        const Vec3 heading = sim.mouse().velocity.normalized();
        REQUIRE((nose - heading).norm() < 1e-6);
    }
    REQUIRE_THAT(sim.mouse().orientation.norm(), WithinAbs(1.0, 1e-12));
}

TEST_CASE("mouse reaches the goal on time and then stops", "[sim]") {
    Simulation sim = make_run();
    int ticks = 0;
    while (!sim.reached_goal() && ticks < 30000) {
        sim.step();
        ++ticks;
    }
    REQUIRE(sim.reached_goal());
    REQUIRE((sim.mouse().position - run_goal).norm() <= apex::goal_radius);

    // Arrives once it is within the radius: (500 - 0.5) m / 20 m/s.
    REQUIRE_THAT(sim.time(), WithinAbs((500.0 - apex::goal_radius) / mouse_speed, 2 * apex::sim_dt));

    const RigidBodyState frozen = sim.mouse();
    const double frozen_t = sim.time();
    sim.step();
    REQUIRE(sim.mouse().position == frozen.position);
    REQUIRE(sim.time() == frozen_t);
}

TEST_CASE("step() flags a ground hit and then stops", "[sim]") {
    // Goal below the floor: the mouse dives from 10 m and crosses z = 0 first.
    Simulation sim = make_run(Vec3{0.0, 0.0, 10.0}, Vec3{0.0, 0.0, -100.0});

    int ticks = 0;
    while (!sim.ground_hit() && ticks < 20000) {
        sim.step();
        ++ticks;
    }
    REQUIRE(sim.ground_hit());
    REQUIRE_FALSE(sim.reached_goal());
    REQUIRE(sim.mouse().position.z() <= 0.0);

    const RigidBodyState frozen = sim.mouse();
    const double frozen_t = sim.time();
    sim.step();
    REQUIRE(sim.mouse().position == frozen.position);
    REQUIRE(sim.time() == frozen_t);
}

TEST_CASE("cat keeps its speed range, nose, and acceleration limit while chasing", "[sim][cat]") {
    Simulation sim = make_chase();
    const apex::SpeedParams sp;
    for (int i = 0; i < 30000 && !sim.captured() && !sim.reached_goal(); ++i) {
        sim.step();
        const RigidBodyState& c = sim.cat();
        REQUIRE(c.velocity.norm() >= sp.min_fraction * cat_speed - 1e-9);
        REQUIRE(c.velocity.norm() <= cat_speed + 1e-9);
        REQUIRE(((c.orientation * Vec3::UnitX()) - c.velocity.normalized()).norm() < 1e-6);
        REQUIRE(sim.cat_command().norm() <= apex::cat_accel_max + 1e-9);
    }
}

TEST_CASE("cat command is zero on the first guidance update", "[sim][cat]") {
    // Observer starts with v_hat = 0, so pursuit sees no LOS swing yet.
    Simulation sim = make_chase();
    sim.step();  // tick 0: guidance runs
    REQUIRE(sim.cat_command().isZero());
}

TEST_CASE("cat command is held between guidance updates", "[sim][cat]") {
    Simulation sim = make_chase();
    for (int i = 0; i <= apex::guidance_every; ++i) {
        sim.step();  // ticks 0..10; tick 10 is the first update with v_hat != 0
    }
    const Vec3 held = sim.cat_command();
    REQUIRE(held.norm() > 0.0);
    for (int i = 1; i < apex::guidance_every; ++i) {
        sim.step();
        REQUIRE(sim.cat_command() == held);
    }
    sim.step();  // tick 10: guidance runs again
    REQUIRE(sim.cat_command() != held);
}

TEST_CASE("cat catches the mouse before it reaches the goal, then stops", "[sim][cat]") {
    Simulation sim = make_chase();
    int ticks = 0;
    while (!sim.captured() && !sim.reached_goal() && ticks < 30000) {
        sim.step();
        ++ticks;
    }
    REQUIRE(sim.captured());
    REQUIRE_FALSE(sim.reached_goal());
    REQUIRE((sim.mouse().position - sim.cat().position).norm() < 2.0);  // within one tick of travel

    const RigidBodyState frozen = sim.cat();
    const double frozen_t = sim.time();
    sim.step();
    REQUIRE(sim.cat().position == frozen.position);
    REQUIRE(sim.time() == frozen_t);
}

TEST_CASE("mouse keeps its speed range, nose, and acceleration limit while evading", "[sim][mouse]") {
    Simulation sim = make_chase();
    const apex::SpeedParams& sp = apex::mouse_speed_params;
    bool dodged = false;
    for (int i = 0; i < 30000 && !sim.captured() && !sim.reached_goal(); ++i) {
        sim.step();
        const RigidBodyState& m = sim.mouse();
        REQUIRE(m.velocity.norm() >= sp.min_fraction * mouse_speed - 1e-9);
        REQUIRE(m.velocity.norm() <= mouse_speed + 1e-9);
        REQUIRE(((m.orientation * Vec3::UnitX()) - m.velocity.normalized()).norm() < 1e-6);
        REQUIRE(sim.mouse_command().norm() <= apex::EvadeParams{}.accel_max + 1e-9);
        if (sim.mouse_dodging()) {
            REQUIRE(sim.mouse_t_go() < apex::EvadeParams{}.t_go_on);
            dodged = true;
        }
    }
    REQUIRE(dodged);  // the chase gets close enough to trigger at least one dodge
}

TEST_CASE("mouse does not dodge while the cat is far away in time", "[sim][mouse]") {
    Simulation sim = make_run();  // cat 100 km behind
    for (int i = 0; i < 1000; ++i) {
        sim.step();
        REQUIRE_FALSE(sim.mouse_dodging());
    }
}

TEST_CASE("both slow toward their juke speeds during a weave", "[sim][speed]") {
    // Run until the mouse has been weaving for 1.5 s. Braking 20 -> 15 at 5 m/s^2 takes
    // 1 s, but the held turn push leans forward as the velocity turns under it,
    // which eats part of the brake, so give it extra time.
    Simulation sim = make_chase();
    double weave_time = 0.0;
    double min_cat = cat_speed;
    bool cat_saw = false;
    for (int i = 0; i < 30000 && !sim.captured() && !sim.reached_goal() && weave_time < 1.5; ++i) {
        sim.step();
        weave_time = sim.mouse_dodging() ? weave_time + apex::sim_dt : 0.0;
        min_cat = std::min(min_cat, sim.cat().velocity.norm());
        cat_saw = cat_saw || sim.cat_sees_weave();
    }
    REQUIRE(weave_time >= 1.5);
    REQUIRE_THAT(sim.mouse().velocity.norm(),
                 WithinAbs(apex::mouse_speed_params.juke_fraction * mouse_speed, 0.1));
    REQUIRE(cat_saw);
    REQUIRE(min_cat < cat_speed - 1.0);  // the cat braked at least a little
}
