# Learning Progress

## CMake scaffold
- Introduced (AI explained): targets (library vs executable), find_package vs FetchContent, header declaration vs compiled definition / linker.
- Learner proposed: include/ as public interface, src/ for .cpp; tests/ and docs/ (math derivations) at top level; src split physics → sensor → gnc; core/types.hpp for shared types.
- Learner chose Option B (single library linked by tests and sim) after AI offered A/B; reasoning: "compiles once and is clean".
- Learner chose system packages (dnf) for Eigen3/Catch2 to save disk space, after AI corrected that FetchContent uses more space.
- Needs reinforcement: why compiled code must reach each executable (confused headers with linking).
- Scaffold built and verified: 2/2 smoke tests pass (Eigen 5.0.1, Catch2 v3 via catch-devel).

- Learner chose FetchContent for all deps, no -Werror, keep include/apex/, add Rerun now — after AI gave pros/cons; no reasoning stated.
- Implemented and verified: FetchContent switch, 2/2 smoke tests pass.

## Point-mass state & state arithmetic
- Introduced (AI explained): circular motion a = v^2/r = -omega^2 * position; f(t,x) as rule of change; derivative shares state shape.
- Learner proposed: state = position xyz + velocity; struct with named fields; operator overloading (option 1 of AI-offered 3).
- Demonstrated reasoning: "1 works for all, 2 gets too messy b/c tied to each struct" — understands generic integrator vs per-struct coupling.
- Needs reinforcement: initially conflated type definition with initial values ("assign random vars"); thought mouse stationary in Part 1.
- Implemented: PointMassState + operators, 4 tests; 6/6 tests pass.

## RK4 interface
- Introduced (AI explained): ways to pass callables (fn pointer, lambda, template, std::function; some may heap-allocate); t = sim clock advanced by dt; dt fixed by spec (0.001 s); f returns state derivative {velocity, acceleration} as vectors.
- Learner first answer: t from time passed (correct); thought dt is derived from velocity/acceleration; offered scalar net-force magnitude m·r·√(ω⁴+α²) as f.
- Needs reinforcement: dt as fixed step vs derived quantity; f as vector state derivative vs scalar force magnitude.
- Introduced (AI explained, on request): stack vs heap; comparison of fn pointer / lambda / template / std::function (std::function may allocate on construction/copy, not call).
- Learner proposed: modify x in place (reason: "while moving it's not really at a new state"). AI noted xₙ must survive until all four k's computed.
- Needs reinforcement: put PointMassState in the f slot — confused state x with rule f.

- Learner chose: f as template parameter (reason: "works anywhere it's passed"); return new state instead of in place (stated reason "in-place adds heap" — incorrect, AI corrected: both are stack copies).
- Needs reinforcement: which operations allocate on heap.

- Implemented (learner-confirmed): include/apex/physics/rk4.hpp (template on f and State, returns new state); tests/test_rk4.cpp (tilted 3D circular orbit, 1 period @ dt=0.001; input-unchanged test). 8/8 tests pass, drift 7.6e-11. Zero-heap test deferred to sim-loop step.

## Atmosphere API
- Learner proposed: air thins with altitude (correct); input altitude (+ temp, then altitude only); output pressure and density — missed speed of sound.
- Introduced (AI explained): 1976 model computes T from altitude via layers/lapse rates; p hydrostatic; ρ = p/(RT); a = √(γRT).

- Learner proposed: object with methods (object.density()) so model can be swapped later; keep T internal ("what would you need temp for?"); asked for options on out-of-range.
- Introduced (AI explained): at(h)→struct vs per-quantity methods; virtual interface vs template for swappable models; T uses (viscosity, sensors); out-of-range options (clamp/throw/optional/extrapolate/assert); 1976 tables start ~-5 km.
- Learner raised: ground hit = loss for both cat and mouse, "needs to be realistic". AI explained ground collision belongs in sim loop, not atmosphere clamp (terrain ≠ sea level). Deferred to sim-loop step.
- Learner asked for recommendation: AI recommended template (matches RK4) and clamp at model limits [-5 km, 86 km].

- Learner approved Implementation checkpoint. Implemented: include/apex/physics/atmosphere.hpp (AtmosphereSample, StdAtmosphere1976::at), src/physics/atmosphere.cpp (7-layer constexpr table, clamp [-5,86] km, geometric→geopotential), tests/test_atmosphere.cpp (sea level, 11/20 km, clamp). Math checked via Python mirror (all within 1.3e-5 of tables). Built + verified on Windows (MSVC 19.51, Ninja, CMake 4.4.4): 11/11 tests pass. Learner switched dev to Windows; AI installed toolchain on request and made warning flags MSVC/GCC portable via generator expression.

## Sim loop / step()
- Learner reviewed Week 1 exit criteria (1/3 passed: RK4 drift). Quaternion derivative in Week 1 math requirements, not started.
- Learner proposed: step() advances sim to next moment in time (correct purpose; no internal order yet).
- Learner proposed order: update position → get atmosphere data → check floor and ceiling. Correct: floor check after move. Needs reinforcement: atmosphere is an input to f (forces at current position), not a post-move step. Missing: advance t. Ceiling meaning unclear (game rule?). Rerun inside/outside unanswered.
- Introduced (AI explained): air lookup lives inside f, called 4× per RK4 step; Part 1 dummy f doesn't use air.
- Learner answered: t increments (AI placed it after RK4 move); ceiling is a physical limit, not a game rule → no ceiling check. Proposed Rerun logging inside step(); AI challenged: Rerun allocates (breaks zero-heap benchmark), 1000 Hz vs ~60 Hz display, tests run without viewer.
- Agreed so far (learner): step() = RK4 move (air inside f) → t += dt → floor check (ground hit = loss).

- Learner stated: project math (scope doc) was vibe-coded; needs to learn it themselves.
- Learner chose math learning "as it comes up".
- AI explained (on request): what a Rerun log call does (copies to buffer → allocates, timestamps, sends to viewer; output only).
- Learner decided: Rerun logging outside step(), every ~10 ticks (100 Hz).

- Learner approved Implementation checkpoint. Implemented: include/apex/sim/simulation.hpp + src/sim/simulation.cpp (Simulation{mouse, center, omega, t, ground_hit}; step() = RK4 → t+=dt → floor z<=0; no-op after hit); apps/sim_main.cpp → apex_sim (Rerun spawn, logs point + growing path every 10 ticks, one period); tests/test_simulation.cpp (operator new counter + sanity check, zero-heap over 1000 steps, clock, ground hit). 15/15 tests pass on MSVC. apex_sim ran (exit 0, viewer spawned); Eye test awaits learner visual confirmation. Rerun Scripts added to user PATH.

- Learner confirmed Rerun circle closed: Benchmark #3 passed. All Week 1 exit criteria met.
- Learner asked whether stack is industry standard; AI explained (C++/CMake/Eigen standard; GoogleTest more common than Catch2; Rerun newer; MATLAB/Simulink gap).

- Learner stated career goal: embedded defense. AI explained relevant gaps (JSF AV C++/MISRA, Linux/GCC cross-compile, hardware/RTOS, guidance math, clearance, DO-178C). Possible future decision: adopt JSF/MISRA-style rules.

- Learner asked why no interceptor; AI explained cat = interceptor, Week 1 scope is mouse-only dummy. Learner chose: add cat in Week 2 together with pursuit logic (no reason stated).

## Quaternion derivative
- Introduced (AI explained): orientation options (Euler/gimbal lock, rotation matrix, quaternion); unit quaternion [cos(θ/2), sin(θ/2)·axis]; ⊗ non-commutative; q̇ = ½ q ⊗ [0, ω] is just another f for RK4.
- Build checkpoint asked: how to handle quaternion norm drift after RK4, and where in code.

## Pending decision
Decision: Build checkpoint open — quaternion norm drift handling (awaiting learner reasoning). Uncommitted Week 1 work suggested for commit.
