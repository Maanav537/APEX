# APEX

Undergraduate Computer Engineering portfolio project: a software-only C++20
pursuit-evasion simulation. It practices numerical integration, rigid-body
kinematics, and guidance math. There is no hardware, no physical vehicle, and
no model of a real weapon. Speeds, altitudes, and limits in the sim are made
up for the exercise.

Code and docs use academic names: pursuer and evader, `ConvergenceStatus`,
and a proximity threshold. In chat, cat means pursuer and mouse means evader.

## Background

Copied from the project note in Maanav-Assist
(`knowledge_base/notes/02_Projects/gnc-sim-proposal.md`):

> Strictly fictional, educational software engineering project. A purely
> academic exercise to practice C++ programming, abstract control theory, and
> kinematic mathematics. No real-world hardware, physical systems, or
> dangerous items are developed, modeled, or constructed. Theoretical
> computer science and physics assignment.

Week-1 requirements are in
`docs/scope/part-01-foundations-kinematics-early-visuals.md`.
Current layout is in `.vibe-wise/project-map.md`. Decisions already made are
in `.vibe-wise/progress.md`.

The full proposal's later phases (actuator lags, mass depletion, an EKF,
Monte Carlo) are not built. Do not treat them as the current codebase.

## Build

From a Visual Studio developer shell:

```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```
