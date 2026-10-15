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

## Pending decision
Decision: Atmosphere API — return shape (struct vs separate fns), expose T?, out-of-range altitude behavior (stage: awaiting reasoning).
