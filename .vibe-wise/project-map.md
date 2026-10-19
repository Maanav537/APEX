# Project Map

## Purpose
APEX: pursuit/evasion kinematics — autonomous pursuer vs evader (3D rendezvous / terminal-state convergence). Scope docs per part in docs/scope/. Academic naming is locked in `include/apex/core/domain.hpp` (pursuer/evader, ConvergenceStatus, proximity threshold); avoid weaponized nouns in symbols and docstrings.

## Requirements
- CMake project scaffold with Eigen3 and Catch2
- RK4 numerical integrator
- 1976 US Standard Atmosphere model
- Point-mass and rigid-body 6-DOF quaternion engine
- Part 1 (docs/scope/part-01-...md): RK4 fixed step 1000 Hz; atmosphere rho/p/a(h); Rerun viewer, circular dummy evader;
  exit: RK4 drift < 1e-6 on circular orbit, zero heap allocs in step(), Rerun draws closed circle.
- Learner decisions vs doc: FetchContent for all deps (matches doc), warnings only / no -Werror (differs), include/apex/core/ (differs), Rerun added now (matches doc).

## Components
- Implemented: `apex` library (CMakeLists.txt) — public headers in include/apex/, sources in src/. Learner choice: single library linked by all executables.
- Implemented: include/apex/core/types.hpp (learner idea; Vec3/Mat3 aliases are AI addition). PointMassState{position, velocity} with +, scalar* (learner: struct + operator overloading; AI: same type holds derivative, header-only inline).
- Implemented: include/apex/core/domain.hpp — PursuerState/EvaderState aliases, ConvergenceStatus, ProximityThreshold, relative_state(pursuer_pos, …), evaluate_convergence.
- Implemented: tests/test_types.cpp, tests/test_domain.cpp.
- Implemented: include/apex/physics/rk4.hpp — header-only `rk4_step(f, x, t, dt)` (learner: template f, return new state; AI: template on State too). tests/test_rk4.cpp.
- Implemented (AI placeholder): include/apex/core/version.hpp, src/core/version.cpp — to be replaced once physics lands.
- Implemented: tests/ → `apex_tests` (Catch2 v3, discovered by ctest), tests/test_smoke.cpp.
- Implemented: include/apex/physics/atmosphere.hpp + src/physics/atmosphere.cpp — StdAtmosphere1976::at(h) → {density, pressure, speed_of_sound}; clamp [-5,86] km (learner: struct, clamp, T internal; AI: template-swap, geopotential, limits).
- Implemented: include/apex/sim/simulation.hpp + src/sim/simulation.cpp — Simulation::step() (learner: RK4 → t+=dt → floor check, no ceiling; AI: class shape, z-up/ground z=0, dummy center-pull f, no-op after hit).
- Implemented: apps/sim_main.cpp → apex_sim (learner: logging outside step every 10 ticks; AI: point + path line strip, one period). tests/test_simulation.cpp.
- Implemented: RigidBodyState{position, velocity, orientation (Quat), angular_velocity (body frame)} + RigidBodyParams{mass, inertia_body} in types.hpp (learner: fields, standalone, constants outside state; AI: body-frame ω, Eigen Quat, coeff-wise ops, params struct). Decided, not yet coded: normalize q in Simulation::step() after RK4.
- Implemented: include/apex/physics/rigid_body.hpp — Wrench + rigid_body_derivative(t, x, params, forces) (learner: four derivative equations, forces passed as function; AI: Wrench frames, precomputed I⁻¹, header-only). tests/test_rigid_body.cpp.
- Implemented: Simulation now flies a RigidBodyState mouse (learner: nose = body +x tangent, ω=(0,0,Ω), τ=0, q₀ = 90° about z, normalize after RK4; AI: forces lambda via Wrench, params 1 kg / identity I, Rerun nose arrow).
- Implemented: include/apex/sim/capture.hpp — closest_distance(r0, r1) over one tick, capture_radius = 0.5 m (learner: s* formula, clamp [0,1], d·d < ε → |r₀|; AI: header-only in sim/ as referee, ε = 1e-12 m², not wired yet). tests/test_capture.cpp.
- Implemented: goal-seeking mouse replaces the circle — Simulation(mouse, params, goal, speed), reached_goal(), goal_radius 0.5 m (learner: straight line, velocity set directly, arrival via closest_distance, replace circle; AI: aim nose via FromTwoVectors, zero-Wrench RK4 coast, freeze on arrival, demo start (0,0,1000) → goal (500,0,1000) at 20 m/s).
- Planned (learner): src/physics, later sensor, gnc; docs/ for math derivations.

## Main Flow
apex_tests --links--> apex --links--> Eigen3
apex_sim --links--> apex, rerun_sdk

## Data and Trust Boundaries
None (local library).

## Build and Deployment
- Dev machine now Windows (learner switched): VS Build Tools 2026 (MSVC, VCTools workload), CMake 4.4.4, Ninja 1.13.2 via winget. Build from "Developer PowerShell for VS": cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release. MSVC chosen by AI because Rerun C++ SDK supports MSVC on Windows (not MinGW).
- Deps via FetchContent (pinned, AI-proposed versions): Eigen 5.0.1, Catch2 v3.16.1, Rerun SDK 0.39.0 (EXCLUDE_FROM_ALL; Arrow builds only when linked). Toolchain: dnf gcc-c++ cmake.
- Rerun viewer: pip install --user rerun-sdk==0.39.0 (installed; rerun.exe in %APPDATA%/Python/Python312/Scripts, not on PATH)
- cmake -S . -B build && cmake --build build && ctest --test-dir build
- C++20; warnings /W4 /permissive- on MSVC, -Wall -Wextra -Wpedantic elsewhere (AI additions)

## Unknowns
Torque/force sources (gravity, thrust, drag, fins).
