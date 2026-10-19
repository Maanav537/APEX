# Week 1: Foundations, Kinematics, & The First Mouse

## 1. Overview: Building the Arena
Welcome to Phase 1. Before the cat can hunt the mouse, we must first build the arena they fly in and establish the rules of physics. 

In this phase, both the "cat" (the autonomous pursuit vehicle) and the "mouse" (the evader) are modeled purely as "dots" in 3D space (point-masses). There is no pursuit logic yet and no dodging. The goal is to ensure your mathematical engine can flawlessly move these dots through space, and to hook up a 3D visualizer so you can actually watch it happen.

## 2. Core Deliverables
*   **Project Architecture (The Toolbox):** Establish the CMake project scaffold. Hook up Eigen3 (for fast matrix math), Catch2 (for testing), and the Rerun C++ SDK (our window into the arena).
*   **Numerical Integrator (The Heartbeat):** Implement a fixed-step 4th-order Runge-Kutta (RK4) integrator operating at $1000\text{ Hz}$ ($\Delta t = 0.001\text{ s}$). This is the engine that calculates exactly where the cat and mouse are every millisecond.
*   **Atmospheric Environment (The Air):** Code the 1976 US Standard Atmosphere model to dynamically calculate air density $\rho(h)$, pressure $p(h)$, and the speed of sound $a(h)$ based on altitude.
*   **Early Visualization (The Security Camera):** Initialize the Rerun viewer. Instead of waiting until Week 4, we will immediately draw our test math as 3D points in space to visually guarantee our code works.

## 3. Mathematical Requirements
You will need to implement the following core algorithms in C++ using Eigen3. Don't worry about the complexity; focus on translating these formulas into clean C++ functions.

**Quaternion Derivative (For when the cat/mouse start turning later):**
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
2.  **Step 2: The Data Structures.** Define your core state vectors for the cat and mouse in `include/core/types.hpp` using Eigen aliases (e.g., `using Vector3d = Eigen::Vector3d;`).
3.  **Step 3: The Heartbeat.** Write the RK4 math engine. 
4.  **Step 4: The Air.** Implement the 1976 Standard Atmosphere equations.
5.  **Step 5: Visualizing the Math.** Initialize a basic Rerun viewer. Inject a "dummy" mouse flying in a perfect, continuous circle into your RK4 integrator. Use `rr::Points3D` to log its coordinates.

## 5. Exit Criteria (Success Metrics)
Before moving to Week 2 (Giving the Cat its Brain), you must pass the following tests:
*   [ ] **The Math Test:** Your RK4 integrator achieves a drift of $< 10^{-6}$ when tested against a known 3D circular orbit in Catch2.
*   [ ] **The Memory Test:** Zero heap (dynamic) memory allocations occur during the active inner simulation loop (`step()`). It must be blazing fast.
*   [ ] **The Eye Test:** Rerun.io successfully opens a 3D window and draws a smooth, closed circular path proving the mouse is flying perfectly.

Cat pursuer design (PN, g-limit, catch): `docs/scope/cat-pursuer-locked-decisions.md`.