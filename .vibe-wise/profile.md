# Learner Profile

Learning mode: active
Onboarding: complete

## Project
Situation: New
Building: APEX, a flight dynamics simulation (CMake/Eigen3/Catch2 scaffold, RK4 integrator, 1976 US Standard Atmosphere, point-mass and rigid-body 6-DOF quaternion engine)
Codebase familiarity: N/A (new project)
Learning scope: entire system

## Experience
Overall programming: Intermediate
Stack familiarity: Beginner (C++, CMake, Eigen3, Catch2, flight dynamics, quaternions, numerical integration)
Math: Beginner for project math — learner says scope doc math was vibe-coded; wants to learn it themselves. Background: calculus done, currently in Engineering Math A

## Goals
Primary: Understand the project end to end as we build it (default)
Capability goal: Career in embedded defense (GNC / real-time C++); learner stated. Resume is drone-focused; wants project to maximize resume value

## Preferences
Checkpoint frequency: Normal
Question style: Open-ended
Implementation style: AI writes code
Math learning: as it comes up — explain each new piece of math before coding it; revisit RK4/atmosphere later. Learner wants the math pieces given separately so they assemble the solution themselves; always state which variable they are solving for

## Strong Concepts
- Minimizing a dot-product quadratic (solved s* for closest approach); clamping s to the tick

## Developing Concepts
- Self dot product d·d = |d|² (confused with angle between vectors)
- Net force ⇒ velocity change, F = m·a (expected constant push to hold speed)

## Revisit
- RK4 math (why k1–k4 and weights work) — implemented but not learned
- 1976 atmosphere formulas (lapse rates, hydrostatic pressure, geopotential altitude) — implemented but not learned
