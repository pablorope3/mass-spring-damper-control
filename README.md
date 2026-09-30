# mass-spring-damper-control

Dynamic simulation and control of a mass-spring-damper system using Python and MATLAB/Simulink. :)

## Overview

This project models a one-degree-of-freedom mass-spring-damper system, simulates its dynamic response, and (in progress) designs and compares different feedback controllers.

**Project status**

- [x] Mathematical model
- [x] Free-response simulation in Python (validated against the analytical solution)
- [ ] P controller
- [ ] PD controller
- [ ] LQR controller
- [ ] MATLAB/Simulink implementation

## Objectives

- Derive the equation of motion and its state-space representation.
- Simulate the system numerically and validate the results against the analytical solution.
- Design and compare P, PD and LQR controllers.
- Reproduce the simulations in MATLAB/Simulink and compare them with the Python results.

## Mathematical Model

The system is described by:

$$
m\ddot{x} + c\dot{x} + kx = F(t)
$$

In state-space form, with $\mathbf{x} = [x,\ \dot{x}]^T$ and input $u = F(t)$:

$$
\dot{\mathbf{x}} = A\mathbf{x} + B u,
\qquad
A = \begin{bmatrix} 0 & 1 \\ -\frac{k}{m} & -\frac{c}{m} \end{bmatrix},
\quad
B = \begin{bmatrix} 0 \\ \frac{1}{m} \end{bmatrix}
$$

Parameters used in the simulations:

| Parameter | Symbol | Value | Unit |
|---|---:|---:|---|
| Mass | $m$ | 1 | kg |
| Spring stiffness | $k$ | 10 | N/m |
| Damping coefficient | $c$ | 1 | Ns/m |

With these values, $\omega_n \approx 3.16$ rad/s and $\zeta \approx 0.158$, so the system is underdamped.

The full derivation is available in [`docs/mathematical_model.md`](docs/mathematical_model.md).

## Simulation

The free response (initial displacement $x(0) = 1$ m, no external force) is solved with `scipy.integrate.solve_ivp` and compared with the analytical solution. The maximum difference between both is around $10^{-10}$ m.

```bash
python src/simulation.py
```

![Free response of the mass-spring-damper system](results/free_response.png)

## Control

The controllers below are planned and will be added step by step. All of them will be tested on the same system and with the same reference.

### P Controller

Proportional feedback on the position error:

$$
u = K_p\,(r - x)
$$

*Status: planned.*

### PD Controller

Proportional feedback on the error plus derivative action (added damping):

$$
u = K_p\,(r - x) - K_d\,\dot{x}
$$

*Status: planned.*

### LQR Controller

Optimal state feedback that minimizes the cost function

$$
J = \int_0^\infty \left( \mathbf{x}^T Q \mathbf{x} + u^T R u \right) dt,
\qquad
u = -K\mathbf{x}
$$

where $K$ is obtained by solving the algebraic Riccati equation.

*Status: planned.*

## Results

Free response: see the plot in the [Simulation](#simulation) section.

Controller comparison (overshoot, settling time, steady-state error and control effort): *coming soon.*

## MATLAB/Simulink

*Coming soon.* A Simulink model of the same system will be added in a `simulink/` folder, and its results will be compared with the Python simulation.

## Installation

Clone the repository:

```bash
git clone https://github.com/pablorope3/mass-spring-damper-control.git
cd mass-spring-damper-control
```

**Option A: conda**

```bash
conda create -n msd python=3.11
conda activate msd
pip install -r requirements.txt
```

**Option B: venv**

```bash
python -m venv .venv
source .venv/bin/activate      # Mac/Linux
.venv\Scripts\activate         # Windows
pip install -r requirements.txt
```

Run the simulation:

```bash
python src/simulation.py
```

The plot is saved in `results/free_response.png`.
