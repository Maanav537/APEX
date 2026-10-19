# Cat pursuer — locked decisions

The **cat** is the pursuer. The **mouse** is the evader. This file is the file pointer for cat design: read it before changing pursuit, PN, forces, or catch logic.

## Rigid body and heading

- The cat is a rigid body.
- Every tick, set the cat’s nose directly along its velocity, the same way as the mouse.

## PN and the integrator

- PN runs at 100 Hz, **outside** RK4.
- The PN command is held constant across RK4’s 4 calls.

## Force and speed

- PN’s acceleration becomes a force: `F = m · a_cmd`.
- After each RK4 step, reset the cat’s speed to a constant. The g-limit does not change that speed; it only caps the acceleration command.

## Catch

- The cat catches the mouse when the closest distance between them **during a tick** is ≤ 0.5 m.

## G-limit (`a_max`)

PN may ask for any acceleration. The g-limit caps the commanded acceleration at `a_max`.

1. **Where:** apply the cap **between PN and the force** (after PN returns `a_cmd`, before `F = m · a_cmd`). Not inside PN, and not on the velocity after integration.
2. **How:** if `|a_cmd| > a_max`, **scale the whole vector** so `|a| = a_max`. Direction stays the direction PN asked for. Do **not** clip x, y, z separately — that bends the vector and the cat turns a different way.
