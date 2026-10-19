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
- Planned (learner): src/physics, later sensor, gnc; docs/ for math derivations.

## Main Flow
apex_tests --links--> apex --links--> Eigen3
future sim exe --links--> apex (not created)

## Data and Trust Boundaries
None (local library).

## Build and Deployment
- Deps via FetchContent (pinned, AI-proposed versions): Eigen 5.0.1, Catch2 v3.16.1, Rerun SDK 0.39.0 (EXCLUDE_FROM_ALL; Arrow builds only when linked). Toolchain: dnf gcc-c++ cmake.
- Rerun viewer: pip install --user rerun-sdk==0.39.0 (must match SDK)
- cmake -S . -B build && cmake --build build && ctest --test-dir build
- C++20, -Wall -Wextra -Wpedantic (AI additions)

## Unknowns
Atmosphere API, sim loop / step() and zero-heap test, 6-DOF design.
