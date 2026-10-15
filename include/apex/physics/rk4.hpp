#pragma once

namespace apex {

// One fixed-step 4th-order Runge-Kutta step.
//
// `f(t, x)` returns dx/dt, which must have the same type as `x`. `State` only
// needs `+` and scalar `*`, so the same integrator serves point-mass and 6-DOF.
// `x` is not modified; the next state is returned.
template <class State, class F>
State rk4_step(F&& f, const State& x, double t, double dt) {
    const double half = 0.5 * dt;
    const State k1 = f(t, x);
    const State k2 = f(t + half, x + half * k1);
    const State k3 = f(t + half, x + half * k2);
    const State k4 = f(t + dt, x + dt * k3);
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

}  // namespace apex
