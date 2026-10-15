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
- Week 1 committed (358db35); .vscode/ left uncommitted.
- Learner asked whether sin/cos is "just the rotation". AI explained: cos(θ/2) and sin(θ/2) are angle components; the axis supplies the direction; half-angle comes from the q v q* double application.
- Learner proposed: get the norm, then divide or subtract. AI showed with a numeric example that divide keeps direction and subtract does not. "How" settled: q / ‖q‖.
- Learner proposed: normalize inside rk4_step "because it adjusts the point mass as well". AI challenged: point mass has no unit-length constraint; normalizing in the generic integrator couples it to one type. Retry asked: f vs Simulation::step().
- Learner decided: normalize q in Simulation::step() after RK4 (no reason stated; AI explained step() is the only place holding the new q). Order: RK4 → normalize → t += dt → floor check. Not implemented yet (no rigid-body state exists).

## Rigid-body state
- Introduced (AI explained): translation vs rotation table (q/ω/torque/inertia tensor I as rotational mass).
- Build checkpoint asked: RigidBodyState fields; reuse PointMassState or standalone; mass/inertia in state or elsewhere.
- Learner proposed: angular velocity, position, force. Missing velocity and orientation q. Needs reinforcement: force is an input computed in f, not state.
- Introduced (AI explained): state test — has a derivative f returns / old value needed for new.
- Learner added torque (AI: also an f input, not state). Learner decided: standalone RigidBodyState, not reusing PointMassState (reason "needs more inputs"; AI noted fields vs inputs). Learner decided: mass not state (correct, constant mass). Learner said inertia is state because it changes; AI explained body vs world frame: I_body constant, I_world = R I_body Rᵀ derived. Velocity and q still not named.
- Learner completed: state = position, velocity, orientation, angular velocity; inertia constant.
- Learner approved Implementation checkpoint (AI additions: ω in body frame, Eigen::Quaterniond, coefficient-wise +/*, RigidBodyParams{mass, inertia_body}). Implemented in include/apex/core/types.hpp; tests in test_types.cpp (defaults, arithmetic) and test_rk4.cpp (RK4 with RigidBodyState). 18/18 pass on MSVC.

- Learner asked what RK4 is and why normalize q. AI explained: ODE dx/dt=f, Euler tangent step, RK4 four slopes with Simpson-like 1-2-2-1 weights; linear sums leave the unit 4D sphere, so q drifts. Explanation only, no demonstrated understanding yet.

## Rigid-body dynamics f
- Introduced (AI, math pieces): p-dot=v, v-dot=F/m; q-dot = ½ q⊗[0,ω] (body ω); cross product; Euler eq I ω̇ + ω×(Iω) = τ; frame change q*v*q⁻¹.
- Build checkpoint asked: write all four derivatives (solve for ω̇); how F and τ get into f extensibly.
- Learner answered: ṗ = v ✓, v̇ = F/m ✓ (dropped dot). q̇ "½ dot product matrix" — confused ⊗ with dot product. ω̇ = τ/(I²ω²) — wrong: folded additive gyro term into denominator, divided by matrix. Needs reinforcement: quaternion product vs dot product; matrix inverse; additive vs multiplicative terms.
- Introduced (AI explained): matrix inverse I⁻¹ (left-multiply). Retry asked for ω̇ via scalar-analog table.
- Learner retry: τ − I·ω̇ / ω×(I·ω) — solved for wrong side, divided by vector. AI gave symbol mapping and worked step 1 (I·ω̇ = τ − ω×(Iω)); step 2 left to learner.
- Learner step 2: (τ − ω×(Iω)) / I — correct idea; AI fixed notation (brackets, I⁻¹ on left). Four derivatives complete: ṗ=v, v̇=F/m, q̇=½q⊗[0,ω], ω̇=I⁻¹(τ−ω×(Iω)).
- Learner asked why I⁻¹. AI explained: non-commutative so no division, matrices mix axes, diagonal I⁻¹ = per-axis division, precompute I⁻¹ once.
- Build checkpoint asked: how F and τ get into f without rewriting dynamics later.

- Learner decided: pass F and τ in as a function (like RK4's f).
- Learner approved Implementation checkpoint (AI additions: Wrench{force world, torque body}, inertia_body_inv precomputed via make_rigid_body_params, header-only rigid_body_derivative in physics/rigid_body.hpp). Implemented + tests/test_rigid_body.cpp (F/m, I⁻¹τ, torque-free principal spin, 90° z-spin with RK4+normalize). 22/22 pass on MSVC.

## Pending decision
Decision: none open. Next: wire rigid body + q normalization into Simulation::step() (sim still uses point-mass mouse), or Week 2 scope (cat + pursuit).
