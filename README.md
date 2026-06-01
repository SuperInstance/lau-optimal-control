# lau-optimal-control

Optimal control theory in Rust — LQR, Riccati equations, Pontryagin's Maximum Principle, Hamilton-Jacobi-Bellman, trajectory optimization, controllability/observability analysis, bang-bang control, and agent action planning.

55 tests covering convergence, stability, and correctness across all modules.

---

## What This Does

| Module | What you get |
|---|---|
| **LQR** | Discrete and continuous LQR with gain matrix K and cost-to-go V(x) = x'Px |
| **Riccati** | DARE, CARE, and differential Riccati equation (finite horizon) |
| **Pontryagin** | Hamiltonian system solver, PMP condition verification |
| **HJB** | Grid-based 1D Hamilton-Jacobi-Bellman solver, LQR value function |
| **Bang-bang** | Double integrator time-optimal control, switching function analysis |
| **Trajectory** | Direct collocation and single shooting with gradient-based optimization |
| **Controllability** | Rank test, Gramian computation, stabilizability/detectability |
| **Dynamics** | Linear and nonlinear system simulation, Jacobian linearization |
| **Agent** | Policy computation, trajectory simulation, action planning, discrete action selection |

---

## Key Idea

Everything builds on `nalgebra` for linear algebra. The LQR solver computes the optimal gain matrix K = R⁻¹B'P where P solves the algebraic Riccati equation. The agent layer wraps LQR into a practical tool: define your agent's dynamics (A, B), specify costs (Q, R), and get a policy that maps any state to the optimal action. Trajectories are simulated forward in time, and the value function gives you the cost-to-go at every point.

The Pontryagin and HJB modules implement the two other major approaches to optimal control — necessary conditions via the maximum principle, and sufficient conditions via dynamic programming.

---

## Install

```toml
[dependencies]
lau-optimal-control = "0.1.0"
```

Or as a git dependency:

```toml
[dependencies]
lau-optimal-control = { git = "https://github.com/SuperInstance/lau-optimal-control" }
```

Requires **Rust 2021 edition**.

### Dependencies

| Crate | Why |
|---|---|
| `nalgebra` | Matrices, vectors, eigenvalues, LU, SVD |
| `serde` | Serialize/deserialize all result types |
| `serde_json` | JSON serialization |

---

## Quick Start

### LQR for a discrete system

```rust
use lau_optimal_control::lqr::solve_dlqr;
use nalgebra::{DMatrix, DVector};

let a = DMatrix::from_row_slice(2, 2, &[1.0, 0.1, 0.0, 1.0]);
let b = DMatrix::from_row_slice(2, 1, &[0.0, 0.1]);
let q = DMatrix::identity(2, 2);
let r = DMatrix::from_row_slice(1, 1, &[0.1]);

let sol = solve_dlqr(&a, &b, &q, &r).unwrap();
let x = DVector::from_vec(vec![5.0, 1.0]);
let u = sol.control(&x);        // u = -Kx
let v = sol.cost_to_go(&x);     // V(x) = x'Px
```

### Agent policy

```rust
use lau_optimal_control::agent::{AgentModel, AgentPolicy};

let agent = AgentModel::new(
    DMatrix::from_row_slice(2, 2, &[1.0, 0.1, 0.0, 1.0]),
    DMatrix::from_row_slice(2, 1, &[0.0, 0.1]),
    vec!["pos".into(), "vel".into()],
    vec!["force".into()],
);

let policy = agent.optimal_policy(
    &DMatrix::identity(2, 2),
    &DMatrix::from_row_slice(1, 1, &[0.1]),
).unwrap();

let x0 = DVector::from_vec(vec![5.0, 0.0]);
let traj = policy.simulate(&x0, 200);
println!("regulated? {}", traj.is_regulated(0.5));
```

### Controllability check

```rust
use lau_optimal_control::controllability::check_controllability;

let a = DMatrix::from_row_slice(2, 2, &[0.0, 1.0, 0.0, 0.0]);
let b = DMatrix::from_row_slice(2, 1, &[0.0, 1.0]);
let result = check_controllability(&a, &b);
assert!(result.controllable);
```

### Bang-bang control

```rust
use lau_optimal_control::bangbang::bangbang_double_integrator;

let result = bangbang_double_integrator(5.0, 0.0, 1.0, 0.01);
// Starts at x=5, v=0 with max force 1.0
// Optimal: full thrust toward origin, then switch to full brake
```

### Trajectory optimization (direct collocation)

```rust
use lau_optimal_control::trajectory::DirectCollocation;

let dc = DirectCollocation::new(2, 1, 50, 0.0, 5.0);
let x0 = DVector::from_vec(vec![1.0, 0.0]);

let result = dc.solve(
    &|x, u| DVector::from_vec(vec![x[1], u[0]]),
    &|x, u| x[0]*x[0] + 0.01*x[1]*x[1] + u[0]*u[0],
    &|x| x[0]*x[0] + x[1]*x[1],
    &x0, None, 200, 0.01,
);
```

### PMP for linear systems

```rust
use lau_optimal_control::pontryagin::solve_pmp_linear;

let result = solve_pmp_linear(&a, &b, &q, &r, &x0, 10.0, 0.01);
// result.states, result.costates, result.controls, result.hamiltonians
```

---

## API Reference

### `lqr` — Linear Quadratic Regulator

| Function | Description |
|---|---|
| `solve_lqr` | Continuous-time LQR via CARE |
| `solve_dlqr` | Discrete-time LQR via DARE |
| `solve_care` | Continuous algebraic Riccati equation (Newton iteration) |
| `lqr_trajectory` | Compute full LQR trajectory for a linear system |
| `trajectory_cost` | Sum x'Qx + u'Ru along a trajectory |
| `kronecker`, `vectorize`, `unvectorize` | Matrix utilities |

**`LqrSolution`**: `.control(x)`, `.cost_to_go(x)`, `.gain()`, `.p`

### `riccati` — Riccati Equation Solvers

| Function | Description |
|---|---|
| `solve_dare` | Discrete ARE: P = A'PA - A'PB(R+B'PB)⁻¹B'PA + Q |
| `solve_care_iterative` | Continuous ARE (delegates to `solve_care`) |
| `solve_dre` | Differential Riccati: backward integration from terminal P(T) = Qf |

### `pontryagin` — Pontryagin's Maximum Principle

| Function | Description |
|---|---|
| `solve_pmp_linear` | Solve Hamiltonian system for LQ problems |
| `hamiltonian_value` | Compute H(x, p, u) = p'f(x,u) + L(x,u) |
| `verify_pmp_conditions` | Check that H is minimized at u_opt via perturbation |

**`PmpResult`**: `states`, `costates`, `controls`, `hamiltonians`

### `hjb` — Hamilton-Jacobi-Bellman

| Type/Function | Description |
|---|---|
| `HjbSolver1D` | Grid-based HJB on [x_min, x_max] × [0, T] |
| `.solve(dynamics, cost, terminal, u_min, u_max, nu)` | Backward time integration |
| `.interpolate(t, x)` | Bilinear value function lookup |
| `.optimal_control(t, x, ...)` | Greedily minimize Hamiltonian |
| `hjb_value_lqr` | Closed-form V(x) = x'Px for LQR |

### `bangbang` — Bang-Bang Control

| Function | Description |
|---|---|
| `bangbang_double_integrator` | Time-optimal for ẍ = u, \|u\| ≤ u_max |
| `bangbang_control` | General bang-bang from switching function |
| `find_switch_times` | Locate zero crossings of switching function |

### `trajectory` — Trajectory Optimization

| Type | Description |
|---|---|
| `DirectCollocation` | Gradient-based collocation with state/control bounds |
| `ShootingMethod` | Single shooting with backward gradient pass |

Both produce `TrajectoryResult`: `states`, `controls`, `times`, `cost`, `iterations`, `converged`.

### `controllability` — System Analysis

| Function | Description |
|---|---|
| `check_controllability` | Rank of [B, AB, ..., Aⁿ⁻¹B] |
| `check_observability` | Rank of [C; CA; ...; CAⁿ⁻¹] |
| `controllability_gramian` | Solve AW + WA' = -BB' |
| `observability_gramian` | Solve A'W + WA = -C'C |
| `is_stabilizable` | All uncontrollable modes stable? |
| `is_detectable` | All unobservable modes stable? |

### `dynamics` — System Simulation

| Type | Key Methods |
|---|---|
| `LinearSystem` | `.step(x, u, dt)`, `.simulate(x0, dt, n, u_fn)` |
| `NonlinearSystem` | `.step(x, u, dt)`, `.linearize(x0, u0, eps)` → (A, B) |

### `agent` — Agent Action Planning

| Type/Function | Description |
|---|---|
| `AgentModel` | Define dynamics (A, B), state/action labels |
| `AgentPolicy` | Computed LQR policy: `.action(x)`, `.value(x)`, `.simulate(x0, n)` |
| `ActionPlanner` | Plan to reach target state |
| `best_discrete_action` | Pick best from finite action set |
| `AgentTrajectory` | `states`, `actions`, `values`, `.is_regulated(tol)`, `.total_cost()` |

---

## How It Works

### LQR (`lqr.rs`)

The discrete LQR minimizes Σ (xₖ'Qxₖ + uₖ'Ruₖ) subject to xₖ₊₁ = Axₖ + Buₖ. The optimal control is uₖ = -Kxₖ where K = (R + B'PB)⁻¹B'PA and P solves the discrete algebraic Riccati equation (DARE). The cost-to-go is V(x) = x'Px, guaranteed positive semi-definite when (A, B) is stabilizable and (A, Q^(1/2)) is detectable.

The continuous version solves the CARE: A'P + PA - PBR⁻¹B'P + Q = 0 using Newton iteration. At each step, the Lyapunov equation is solved via Kronecker product vectorization.

### Riccati Equations (`riccati.rs`)

- **DARE**: Solved iteratively: Pₖ₊₁ = A'PₖA - A'PₖB(R + B'PₖB)⁻¹B'PₖA + Q. Converges monotonically from P₀ = Q.
- **CARE**: Newton's method on the residual R(P) = A'P + PA - PBR⁻¹B'P + Q. Each Newton step solves a Lyapunov equation.
- **DRE**: Integrate dP/dt = A'P + PA - PBR⁻¹B'P + Q backward from P(T) = Qf. The finite-horizon gain varies with time: K(t) = R⁻¹B'P(t).

### Pontryagin's Maximum Principle (`pontryagin.rs`)

PMP gives **necessary conditions** for optimality. The Hamiltonian H(x, p, u) = p'f(x,u) + L(x,u) must be minimized over u at each time. The state and costate evolve as:

```
dx/dt = ∂H/∂p = f(x, u*)
dp/dt = -∂H/∂x
```

For LQ problems, the optimal control is u* = -R⁻¹B'p. The solver integrates the Hamiltonian system forward using the DRE to initialize the costate.

### HJB (`hjb.rs`)

The HJB equation gives **sufficient conditions** for optimality via dynamic programming. The 1D grid solver discretizes (t, x) space and integrates backward in time:

```
V(t, x) = V(t+dt, x) + dt · min_u { L(x,u) + ∂V/∂x · f(x,u) }
```

At each grid point, the minimization over u is performed by brute-force search over a discretized control grid. Spatial derivatives use central differences. The terminal condition is V(T, x) = ψ(x).

### Bang-Bang Control (`bangbang.rs`)

For minimum-time problems with bounded controls, the optimal control switches between extremes (u_min and u_max). For the double integrator ẍ = u, the switching curve in the (x, v) phase plane is:

```
v = -sign(x) · √(2·u_max·|x|)
```

Above the curve: u = -u_max. Below: u = +u_max. At most one switch.

### Trajectory Optimization (`trajectory.rs`)

- **Direct Collocation**: Discretize the trajectory into N segments. Initialize states linearly between x0 and xf. Iteratively: (1) forward simulate using current controls, (2) backward pass computing adjoint variables, (3) update controls via gradient descent on total cost. Supports state and control bounds.
- **Single Shooting**: Parameterize the problem by the control sequence only. Forward simulate from x0, then compute gradients via backward adjoint pass. Simpler but can diverge for unstable systems.

Both use central finite differences to compute Jacobians of the dynamics and gradients of the cost.

### Controllability (`controllability.rs`)

A linear system (A, B) is controllable if the controllability matrix C = [B, AB, A²B, ..., Aⁿ⁻¹B] has full row rank (rank = n). The rank is computed via SVD with a threshold of 1e-10 on singular values.

The **Gramian** Wc = ∫₀^∞ exp(At)BB'exp(A't)dt measures "how controllable" each direction is. It's computed by solving the Lyapunov equation AW + WA' = -BB' via Kronecker vectorization. A system is controllable iff Wc is positive definite.

**Stabilizable**: All uncontrollable modes are stable. **Detectable**: Dual of stabilizable for (A', C').

### Dynamics (`dynamics.rs`)

Linear systems use `nalgebra` matrix-vector operations. Nonlinear systems are defined by closures and support Jacobian linearization via central finite differences: A_ij = (fᵢ(x+εeⱼ) - fᵢ(x-εeⱼ)) / (2ε).

---

## The Math

### Algebraic Riccati Equation (CARE)

For continuous-time LQR with cost ∫(x'Qx + u'Ru)dt, the optimal feedback is u = -Kx where K = R⁻¹B'P and P solves:

```
A'P + PA - PBR⁻¹B'P + Q = 0
```

P is the unique positive semi-definite solution. Newton's method iterates:

```
(A-BKₖ)'ΔP + ΔP(A-BKₖ) = -(A'Pₖ+PₖA-PₖBR⁻¹B'Pₖ+Q)
Pₖ₊₁ = Pₖ + ΔP
```

Each step solves a Lyapunov equation (linear in ΔP), converging quadratically near the solution.

### Discrete Riccati Equation (DARE)

For xₖ₊₁ = Axₖ + Buₖ with cost Σ(x'Qx + u'Ru):

```
P = A'PA - A'PB(R + B'PB)⁻¹B'PA + Q
```

The gain K = (R + B'PB)⁻¹B'PA ensures the closed-loop A - BK has all eigenvalues inside the unit circle.

### Pontryagin's Maximum Principle

**Statement**: If u*(t) is optimal, then there exists a costate p(t) such that:

1. **State equation**: dx/dt = ∂H/∂p = f(x, u*)
2. **Costate equation**: dp/dt = -∂H/∂x
3. **Minimum principle**: H(x, p, u*) ≤ H(x, p, u) for all admissible u
4. **Transversality**: p(T) = ∂ψ/∂x (for free-endpoint problems)

For LQ problems, condition 3 gives u* = -R⁻¹B'p analytically.

### HJB Equation

The value function V(t, x) = min_u J(t, x, u) satisfies:

```
-∂V/∂t = min_u { L(x,u) + ∇V · f(x,u) }
```

with terminal condition V(T, x) = ψ(x). For LQR, the solution is quadratic: V(t,x) = x'P(t)x where P solves the DRE.

### Bang-Bang Optimality

For the minimum-time problem with ẍ = u, |u| ≤ u_max, the switching function σ(t) = p₂(t) (the costate associated with velocity). The optimal control is:

```
u*(t) = +u_max if σ(t) < 0
u*(t) = -u_max if σ(t) > 0
```

Since σ crosses zero at most once for the double integrator, the optimal strategy is: full thrust in one direction, then full thrust in the opposite direction (at most one switch).

### Controllability Gramian

The Gramian Wc = ∫₀^∞ exp(At)BB'exp(A't)dt satisfies the Lyapunov equation AWc + WcA' = -BB' (for stable A). Its eigenvalues measure controllability energy: the minimum control energy to reach state x is x'Wc⁻¹x.

---

## License

MIT
