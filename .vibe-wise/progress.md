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


## Sim → rigid body (Option A)
- Learner chose Option A (switch sim to rigid body) over Week 2 pursuit; no reason stated. Rigid-body work committed a782d13.
- Introduced (AI explained): orientation independent of path (force vs ω/torque).
- Learner decided: nose tangent to circle (follows velocity); replace PointMassState with RigidBodyState in Simulation ("new scope: moving rigid body").
- Follow-up asked: ω (axis, rate), τ, and initial q (nose +y vs identity +x).
- Learner: rate ω = 2π/T ✓ (one turn per lap → Ω). Gave mv²/r for axis — conflated path force with rotation axis. AI explained right-hand rule; asked axis sign and torque.
- Learner: axis +z ✓; τ = 0 ✓ but reason wrong ("not spinning unless moving"). AI explained: torque changes spin, constant ω needs none; gyro term zero on principal axis. Needs reinforcement: torque ↔ change in ω (rotational Newton's 1st). Asked initial q.

- Learner answered initial q with "(body y+)": define nose as body +y so identity q already points along world +y (valid; avoids computing q₀). AI flagged: aerospace convention is body +x forward; later thrust/drag/fins depend on the nose axis. Asked learner to choose: body +y nose, or body +x nose with a 90°-about-z q₀.

- Learner decided: nose = body +x (aerospace convention); no reason stated. q₀ numbers asked.

- Learner set up q₀ correctly: a=[0,0,1], q=[cos45°, sin45°·a] → [√2/2, 0, 0, √2/2] (left unevaluated). AI noted Eigen Quaterniond(w,x,y,z) constructor order vs coeffs() storage order (x,y,z,w).
- Implementation checkpoint offered: Simulation → RigidBodyState + params, forces lambda (F = -mΩ²(p-c), τ=0), RK4 → normalize → t → floor; q₀ = 90° about z, ω=(0,0,Ω); tests nose-tangent + |q|=1; Rerun nose arrow.

- Learner approved Implementation checkpoint. Implemented: Simulation holds RigidBodyState + RigidBodyParams; step() = RK4(rigid_body_derivative, forces F=-mΩ²(p-c), τ=0) → normalize q → t += dt → floor. sim_main q₀ via π/4, ω=(0,0,Ω), Rerun nose arrow (mouse/nose). Tests: nose ∥ velocity every 100 ticks over a lap (<1e-6), ‖q‖=1 (1e-12), existing sim tests ported. 24/24 pass on MSVC. apex_sim ran (exit 0); visual nose check awaits learner.

- Learner confirmed nose arrow looks good in Rerun. Asked for an actual visible body. AI explained: Rerun primitives vs mesh (Asset3D), shape logged once in body frame + per-tick Transform3D(translation, q). Visual only, no physics change.

- Learner deferred visible body to later (choices still open: primitive vs mesh, mouse look).

## Week 2: cat + pursuit
- No Week 2 scope doc exists (Part 1 doc names it "Giving the Cat its Brain").
- Build checkpoint asked: how should the cat chase — what it knows, what it controls, the rule turning one into the other.

- Learner asked for the most resume-relevant approach (embedded defense). AI recommended (on request): seeker-based interceptor, g-limited lateral accel command, Proportional Navigation a = N·Vc·λ̇; staged plan 1) PN on truth 2) noisy seeker 3) Kalman filter; truth sim vs flight code separation. Explained PN concept (LOS rate → 0 = collision course).
- Learner decided: staged plan as proposed (reason: resume value). No demonstrated PN understanding yet.
- Stage 1 Build checkpoint asked: cat point mass vs rigid body; where PN runs (inside f 4×/step vs once per tick, held); catch condition.

- Learner answered: 1) rigid body ("why wouldn't it be able to do all?") 2) PN inside f 3) caught = collides with mouse.
- AI challenged: 1) viable, but PN outputs accel, so how it becomes motion in a rigid body is unanswered (no fins/aero yet; force directly vs attitude follows); 2) inside f is fine for perfect truth but breaks at Stage 2 (random noise differs across RK4's 4 calls) and mismatches discrete flight software; 3) points never exactly coincide and fast closing can skip past between ticks; needs a radius (real: lethal radius/proximity fuze, miss distance).

- Learner asked: cat as missile or drone (resume is drone-tailored); how much rewrite? AI explained: core (RK4, atmosphere, rigid body, sim loop) vehicle-agnostic; drone changes Wrench source (rotor thrust along body axis + mixer torques), adds gravity, attitude matters (tilt to accelerate) and cascaded control (guidance → attitude → motors); PN still applies (counter-UAS interceptor drones).

- Learner asked AI to choose; AI recommended interceptor drone (resume fit, counter-UAS demand, cascade = PX4-style embedded skills, uses quaternion work). Learner chose: interceptor drone (via picker; reason = AI's).

- Learner changed decision: cat = missile interceptor, mouse = Shahed-type one-way-attack fixed-wing drone (supersedes interceptor-drone choice; no reason stated). AI explained: less work than drone-cat (no rotor cascade); missile = PN accel + g-limit (+ optional autopilot lag); mouse kinematic waypoint cruise first, fixed-wing aero later. Tradeoff: drone-control (PX4 cascade) resume link drops.

- Learner decided: PN outside RK4, once per tick (1000 Hz), command held across RK4's 4 calls. Stated reason "so it's not perfect data" — incorrect: placement doesn't change data quality (Stage 1 still truth); AI explained real reason = one consistent command per step (noise-safe in Stage 2) + discrete flight-software model. Needs reinforcement: where data imperfection comes from (sensor model, not call placement).

- Scope change: project continues as an abstract, generic pursuit–evasion sim (cat/mouse, made-up non-weapon numbers); not modeled on real weapons or real attack drones. Learner chose this path.

- Learner decided: controller at 100 Hz (every 10 ticks). Reason: faster is smoother but costs more CPU (correct). AI added: slower also means staler command (up to 10 ms hold) → lag can hurt stability.

- Learner decided: capture radius 0.5 m (no reason). Between-tick idea: "check if their paths ever crossed". AI challenged: paths can cross at different times; must compare positions at the same time → closest approach of relative position over the tick. Gave pieces: r(s) = r0 + s·d, s∈[0,1], minimize |r(s)|²; asked learner to solve for s*.

- Learner solved s* = −(r₀·d)/(d·d) correctly (derivative of quadratic = 0). Demonstrated: differentiating a dot-product quadratic.
- Math visuals: learner asked for rendered equations; manimgx setup blocked (auto mode), so AI publishes docs/math-board.html as an artifact per math step, lightweight.

- Learner proposed sim_dt 1000 → 500 Hz ("more realistic"). AI explained: physics dt is numerical accuracy, not realism; realism applies to flight-code rates. Learner decided: physics stays 1000 Hz, controller at a realistic mid-range rate rather than the top (500) → keeps the earlier 100 Hz choice. No code change.

- Learner answered s* edge cases: s<0 → closest already happened, clamp to 0 (correct). d·d = 0 → "angle between them is 0, they already hit" (incorrect). AI explained: d·d = |d|² (self-dot, θ always 0); zero only when d = 0 → gap constant over tick (equal velocities), not a hit; divide-by-zero → NaN; any s gives |r₀|. Needs reinforcement: self-dot product = squared length.

- Learner answered: s* > 1 → hasn't happened yet, clamp to 1 (correct). d·d guard: first thought (a) floor divisor at ε, switched to (b) branch "i guess" after AI listed options; picked s = 0 → |r₀| (correct). AI noted (a) also valid. Proceeding with (b).
- Implemented (learner authorized): include/apex/sim/capture.hpp closest_distance(r0, r1) + capture_radius 0.5 m; tests/test_capture.cpp (4 cases). Not wired into step() (no cat yet). 28/28 tests pass.

- Mouse path: learner first said "react to the cat" + "go for a stationary target"; AI separated goal-seeking (non-reactive) from evasion. Learner decided: goal-only first "so we can test the cat alone" (correct); straight line (hypotenuse = shortest path).
- Learner proposed force toward goal with constant speed — incorrect (net force changes velocity; sideways component curves path). AI explained F = m·a; learner then listed thrust-balanced-by-drag or set velocity; chose set velocity directly (reason "simpler"; AI added: exact known path). Arrival: reuse closest_distance (learner; AI corrected "between 0–1" → distance ≤ radius). goal_radius 0.5 m, "consistent" with capture. Replace circle ("just a dummy").
- Implemented (learner authorized): Simulation(mouse, params, goal, speed) aims velocity + nose at goal each tick, RK4 coast with zero Wrench, freeze on ground hit or reached_goal(); goal_radius in capture.hpp; test_simulation rewritten (line, speed, nose, arrival ≈ 24.975 s, ground dive); sim_main draws goal + straight path. 29/29 tests pass, no project warnings. sim_main built, not run.
- Needs reinforcement: net force ⇒ velocity change (F = m·a).

- Stage 1 Build checkpoint asked: how PN accel becomes rigid-body motion (force vs velocity, nose, g-limit, speed).
- Learner proposed: reset cat speed to constant after each RK4 step (valid; AI added: PN ⊥ LOS has along-velocity component, finite-step lateral force grows |v| by (a·dt)²; force F = m·a_cmd implied). Said "it starts twisting orientation-wise" — ambiguous; AI asked: nose follows velocity (set vs torque) or flagging a problem? Plus where g-limit goes.

- Learner decided: option 1 — set cat orientation directly each tick like the mouse (nose = body +x along v). No reason stated.

- g-limit Build checkpoint: learner decided cap goes between PN and force (no reason stated); scale whole vector (correct choice). Stated reason "whole system slows down rather than just one velocity" — confuses acceleration with velocity (cap limits turn rate, speed is reset to constant anyway). Direction effect not yet explained by learner; AI gave worked example on math board (a_cmd = (300,0,400), a_max = 100) for learner to compute clip vs scale.

- Learner asked for help on clip-vs-scale; AI gave per-component clip worked for x, scale steps, ratio = direction. Learner computed clip (100,0,100), |a|=500, k=0.2, scaled (60,0,80), ratios 1 vs 1.33, clipped "flatter" (all correct, with scaffolding). AI explained: check = does limiter preserve PN's direction; scale keeps it, clip tilts it.

- Learner asked what clip/scale are for; AI explained g-limit purpose (PN unaware of airframe limits). Cap placement: learner stated flow (force uses capped accel); raw force → can't limit sharp turns (correct); velocity cap → garbled (said limits how fast). AI explained velocity cap is a no-op with constant speed; r = v²/a, so cap on accel = minimum turn radius.
- Needs reinforcement: acceleration vs velocity (recurring).

- Learner proposed high-speed real-interceptor-scale numbers (750 m/s, ~38 g, mouse 300 m/s, 5–10 km); AI declined (keeps agreed abstract, non-weapon scale); explained realism = physics/ratios, not speed.
- Learner decided: cat v = 60 m/s, a_max = 100 m/s², computed r_min = 36 m (correct; first attempt had v=50 → 36, self-corrected). Limiter complete.
- PN inputs Concept given: r, v_rel, Vc = −(r·v_rel)/|r|, Ω = (r×v_rel)/(r·r). Asked learner to compute Vc, Ω for head-on case r=(100,0,0), v_rel=(−60,0,0).

- Learner asked what Ω and Vc are (read Ω as "ohm"); AI explained with string analogy. Learner computed Vc = 60, Ω = 0 (correct).
- PN command Concept given: a = N·Vc·(Ω×r̂), r̂ = r/|r|, N = gain. Asked learner to pick N (3–5) with reason.

- Learner asked what N is; AI explained as steering gain with table. Learner decided N = 3 ("so it doesn't over-correct often"; correct tradeoff, AI added slower swing fix).
- Implementation checkpoint approved (Implement this step): gnc/pn.hpp (pn_command, limit_accel), truth at tick start, |r|≈0 guard, capture before goal + captured(), no gravity, cat spawn (−200,−300,1000) +x 60 m/s, tests, Rerun cat path.

- Implemented (authorized): gnc/pn.hpp, Simulation with cat, sim_main cat drawing, test_pn.cpp (6), cat sim tests. 40/41 pass. FAIL: cat misses (closest 237.6 m at 7.6 s). Cause: true PN ⊥ LOS + speed reset discards along-velocity part; weak turn, Vc goes negative. Scratch test (not in project): pure PN a = N·(Ω×v_cat) captures at 8.19 s, miss 0.476 m (marginal vs 0.5 m).

- Learner decided (instead of A/B): raise N from 3 to 5 ("change n=5"; no reason stated). Implemented cat_nav_gain = 5.0. 41/41 pass on MSVC (capture test now passes). Committed + pushed as 13458f5 (learner asked), then learner said "dont commit" — asked whether to revert. Learner reason: "higher N turns harder so it makes up for lost turns" (correct; limited by 100 m/s² cap). Asked: cost of N=5 (cap saturation, noise sensitivity in Stage 2).
- Learner: "hits the cap more often" (true in general). Said "push it" — commit already pushed, kept. Scratch comparison: N=3 miss 237.6 m; N=4 capture 10.16 s miss 0.448; N=5 capture 8.90 s miss 0.497 m (vs 0.5 radius); 0% capped all N; 2 turn flips all N (no extra overshoot).
- Correction: "0.003 m margin" was a measurement artifact (capture fires on first tick under 0.5 m); with capture disabled, all capturing cases pass at ~0.00 m.
- Learner decided: option A ("keeps things clean" — AI tied it to: command ⊥ own velocity only turns, matches constant-speed design). Implementation checkpoint approved: A with N = 5, no commit.
- Implemented: pn_command(r, v_rel, v_pursuer, N) = N·(W × v_pursuer); simulation passes cat_.velocity; test_pn updated (⊥ own velocity). 41/41 pass on MSVC. Not committed.
- Learner pasted a spec matching option A; AI added "need not be ⊥ LOS" test, academic names in test_pn comments. 42/42 pass. Not committed.
- Stage 2 Build checkpoint asked: what seeker measures, where noise added, where it sits. Learner: accel, orientation, angle, velocity; noise added to sensor output; Gaussian (both correct). Confused own-ship IMU quantities with seeker (target) measurements. AI explained seeker vs IMU, Gaussian σ, λ̇ from noisy differencing. Asked: IR (angles-only) vs radar (angles+range); what PN receives instead of truth. Pending: commit Stage 1? CLAUDE.md 60-line deletion intentional?
- Learner chose radar: angles + range. Reason: "without it it would be difficult to track." Viable choice. Range supplies |r| in W = (r × v_rel)/(r·r); angles-only still follows direction and never produces that length. "Difficult to track" does not match that need.
- Learner: PN receives the sensor outputs (instead of truth). Boundary is right. Gap: one sample is range + two angles, which determine r; pn_command also takes v_rel and v_cat. v_cat is own-ship, not a seeker output. Asked how this tick plus the previous tick become r and v_rel.
- Learner asked for a background so Claude would stop flagging the repo. CLAUDE.md had been emptied in 7eb6233 (the prior 60-line FSW brief). Restored a short factual background from Maanav-Assist `gnc-sim-proposal.md` scope note: software-only academic pursuit-evasion, no hardware, made-up numbers, pursuer/evader names. Did not restore the "never lecture / silently remap nouns" rule from that deleted brief. Seeker question still open.
- Learner: v_rel from differencing at the 100 Hz controller period (dt = 0.01 s; correct). Said "it drifts", then clarified they meant noise. AI explained differencing amplifies noise (√2·σ/dt ≈ 141 m/s for σ = 1 m) and that this is different from drift. Learner proposed "a filter" with no type yet. Pending: filter type, what it smooths, its lag cost; r_prev storage; first-tick case.
- Learner did not know filter types; AI explained moving average, exponential, alpha-beta (math board v3). Learner decided: alpha-beta, estimates both r and v_rel (correct). Reason "interception needs to be precise" is incomplete: lag also costs precision, so the real trade is noise vs lag. Did not say what the filter remembers (r̂, v̂). Pending: where the filter state lives, first-tick init, α/β values, σ.
- Learner decided: first tick r̂ = first reading, v̂ = 0 (valid; AI noted that v̂ = 0 makes W = 0, so there is no PN turn until v̂ converges). Filter state lives in a new seeker/filter object (no reason stated). AI explained α/β roles and β = α²/(2−α). Pending: σ, α values.
- Learner decided: σ = 1 m, α = 0.2, computed β = 0.04/1.8 = 1/45 (correct). AI estimate: filtered v̂ noise ≈ 3.7 m/s vs 141 raw. Implementation checkpoint approved, with AI additions: σ_angle = 1 mrad, world-frame az/el, mt19937 fixed seed, tests.
- Implemented (authorized): gnc/seeker.hpp + src/gnc/seeker.cpp (SeekerParams, Seeker::update/r_hat/v_hat/last_reading); Simulation owns Seeker, PN uses r̂/v̂; tests/test_seeker.cpp (5); test_simulation: new "zero on first update" test, held-command test starts at tick 10. NOT BUILT OR RUN: auto mode blocked the build command. Learner's reason for a separate seeker object still unstated.
- Session 2026-10-20: built Stage 2 on Linux (GCC, Release, scratch build dir; not the Windows build/). 47/48 pass: seeker tests and cat capture with seeker r̂/v̂ pass. FAIL: "allocation counter sees heap allocations" (0 == 1). Likely cause: GCC removes the paired new int / delete at -O2 (permitted since C++14), so the counter guard sees nothing. Unrelated to the seeker. Not yet run on MSVC. Pending: learner's fix choice for the guard test; learner's reason for a separate seeker object.
- Learner asked AI to fix the guard test directly (skip). AI made the pointer `int* volatile` so the new/delete pair can't be elided. GCC Release: 48/48 pass, no warnings. Not run on MSVC. Not committed. Learner's reason for a separate seeker object still unstated.

## Week 2: dodging mouse (before Stage 3 Kalman)
- AI explained alpha-beta vs Kalman (fixed vs computed gains; P, Q, R; startup; steady-state equivalence; EKF only if measurements stay polar).
- Learner decided order: dodging mouse first, then Kalman filter (no reason stated).
- Build checkpoint asked: what the mouse knows about the cat, when it dodges, what the dodge is, limits, goal after dodging.
- Learner decided: mouse reads own noisy sensor; reacts always; dodge = turn perpendicular to cat (LOS); own cap 50 m/s² (r = 8 m vs cat 36 m); goal + dodge summed then capped; dodge priority grows as cat gets closer (shape unstated); dodge directions "left/right/up/down, all directions" (no per-tick rule yet). Wants a close contest, not one side OP.
- Learner ideas (later / not yet scoped): fuel limit so the mouse can't run forever; run time limit; an interception/outcome object reusing closest_distance.
- Pending: rule picking one dodge direction per tick; weight-vs-distance shape.
- Learner direction rule: exclude the side the cat comes from; of the remaining (right/up/down), pick the one needing least force ("up/down harder through atmosphere"). AI: toward-cat is already excluded because dodge ⊥ LOS; the sim has no gravity or drag, and density barely changes over metres (climb cost would come from gravity); with a fixed cap every direction gets the same accel, so "least force" saves mouse effort but doesn't target the cat. Asked for a criterion based on what hurts the cat.
- Learner priority rule: dodge when the cat is approaching fast (closing speed); keep for the goal if "it can close a few meters before it gets to the mouse" (ambiguous). AI explained time-to-go t_go = R/Vc; asked learner to restate the switch rule with it.
- Learner decided: dodge direction = opposite the cat's turn (good: forces cat to reverse its turn); dodge on when t_go < 3 s. Alt idea "check how long you can outrun the cat" — AI: mouse 20 vs cat 60 m/s can't outrun, t_go rule kept. AI noted cat needs ~π·36/60 ≈ 1.9 s for a half-turn, relevant to the 3 s choice.
- Pending: how the mouse observes the cat's turn from its noisy sensor (it has no access to the cat's command).
- Learner: turning changes speed ("slows = turning, speeds up = targeting") — incorrect; cat speed is constant, a turn changes velocity direction. AI explained a = Δv/Δt with worked example. Needs reinforcement: acceleration vs velocity (recurring).
- Learner: differencing v̂ again → much noisier, so smooth more (correct; AI added lag cost vs cat's ~1.9 s reversal). AI explained alpha-beta-gamma.
- Learner decided: mouse uses an alpha-beta-gamma filter ("keeps things easier to estimate"); turn-estimate lag < 1 s. 3 s threshold kept (not revisited).
- Session note: an auto-mode safety check blocked tool calls mid-session; learner switched out of auto mode.
- Pending: how the mouse steers toward its goal under a 50 m/s² cap (velocity no longer set directly); α, β, γ values.
- Learner decided: goal steering = push toward goal direction, mouse speed held constant; filter α = 0.2. Not implemented: assistant stopped work on the dodge feature in this session after a safety-classifier stop. Resume in a fresh session.
