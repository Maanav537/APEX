# APEX

APEX is a C++ **cat/mouse** pursuit–evasion simulation. The **cat** is the pursuer. The **mouse** is the evader. Use those names in code, comments, and discussion.

## Read first

1. `.vibe-wise/project-map.md` — layout and current components
2. `.vibe-wise/progress.md` — what the learner already decided
3. `docs/scope/` — part specs and locked design

## Locked cat design

See `docs/scope/cat-pursuer-locked-decisions.md`. Short version:

- Rigid body; nose along velocity every tick (same as the mouse)
- PN at 100 Hz, outside RK4; command held across RK4’s 4 calls
- PN acceleration becomes a force: `F = m * a_cmd`
- After each RK4 step, cat speed is reset to a constant
- Catch when closest distance to the mouse during a tick is ≤ 0.5 m
- G-limit sits **after PN, before the force**: if `|a_cmd| > a_max`, **scale the whole vector** so `|a| = a_max`. Do not clip x/y/z separately.
