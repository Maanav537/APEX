# Week 1: Foundations, Kinematics, & The First Evader

## 1. Overview: Building the Arena
Welcome to Phase 1. Before the pursuer can achieve 3D rendezvous with the evader, we must first build the arena they fly in and establish the rules of physics.

In this phase, both the pursuer (the autonomous 6-DOF flight vehicle, still a point-mass for now) and the evader (a non-cooperative aerial object / dynamic reference point) are modeled purely as "dots" in 3D space. There is no guidance law yet and no evasive maneuvering. The goal is to ensure your mathematical engine can flawlessly move these dots through space, and to hook up a 3D visualizer so you can actually watch it happen.

Use academic pursuit-evasion naming throughout (`pursuer_pos`, `evader_pos`, `ConvergenceStatus`, proximity threshold). Avoid weaponized nouns in symbols and docstrings; the underlying RK4 / rigid-body math is identical either way.

## 2. Core Deliverables
*   **Project Architecture (The Toolbox):** Establish the CMake project scaffold. Hook up Eigen3 (for fast matrix math), Catch2 (for testing), and the Rerun C++ SDK (our window into the arena).
*   **Numerical Integrator (The Heartbeat):** Implement a fixed-step 4th-order Runge-Kutta (RK4) integrator operating at $1000\text{ Hz}$ ($\Delta t = 0.001\text{ s}$). This is the engine that calculates exactly where the pursuer and evader are every millisecond.
*   **Atmospheric Environment (The Air):** Code the 1976 US Standard Atmosphere model to dynamically calculate air density $\rho(h)$, pressure $p(h)$, and the speed of sound $a(h)$ based on altitude.
*   **Early Visualization (The Security Camera):** Initialize the Rerun viewer. Instead of waiting until Week 4, we will immediately draw our test math as 3D points in space to visually guarantee our code works.

## 3. Mathematical Requirements
You will need to implement the following core algorithms in C++ using Eigen3. Don't worry about the complexity; focus on translating these formulas into clean C++ functions.

**Quaternion Derivative (For when the pursuer/evader start turning later):**
$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \mathbf{\omega} \end{bmatrix}$$

**RK4 Integration (Moving the dots forward in time):**
Given a state vector $\mathbf{x}$ (like position and velocity):
$$\mathbf{k}_1 = f(t_n, \mathbf{x}_n)$$
$$\mathbf{k}_2 = f(t_n + \frac{\Delta t}{2}, \mathbf{x}_n + \frac{\Delta t}{2} \mathbf{k}_1)$$
$$\mathbf{k}_3 = f(t_n + \frac{\Delta t}{2}, \mathbf{x}_n + \frac{\Delta t}{2} \mathbf{k}_2)$$
$$\mathbf{k}_4 = f(t_n + \Delta t, \mathbf{x}_n + \Delta t \mathbf{k}_3)$$
$$\mathbf{x}_{n+1} = \mathbf{x}_n + \frac{\Delta t}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)$$

## 4. Development Steps (The "Crawl" Phase)
1.  **Step 1: The Build System.** Create `CMakeLists.txt`. Use `FetchContent` to pull in Eigen3, Catch2, and `rerun-sdk`. Ensure the project compiles with strict warnings (`-Wall -Wextra -Werror`).
2.  **Step 2: The Data Structures.** Define your core state vectors for the pursuer and evader in `include/apex/core/types.hpp` using Eigen aliases (e.g., `using Vector3d = Eigen::Vector3d;`). Role aliases and `ConvergenceStatus` live in `include/apex/core/domain.hpp`.
3.  **Step 3: The Heartbeat.** Write the RK4 math engine.
4.  **Step 4: The Air.** Implement the 1976 Standard Atmosphere equations.
5.  **Step 5: Visualizing the Math.** Initialize a basic Rerun viewer. Inject a "dummy" evader flying in a perfect, continuous circle into your RK4 integrator. Use `rr::Points3D` to log its coordinates.

## 5. Exit Criteria (Success Metrics)
Before moving to Week 2 (Pursuer Guidance / Relative-State Observation), you must pass the following tests:
*   [ ] **The Math Test:** Your RK4 integrator achieves a drift of $< 10^{-6}$ when tested against a known 3D circular orbit in Catch2.
*   [ ] **The Memory Test:** Zero heap (dynamic) memory allocations occur during the active inner simulation loop (`step()`). It must be blazing fast.
*   [ ] **The Eye Test:** Rerun.io successfully opens a 3D window and draws a smooth, closed circular path proving the evader is flying perfectly.
