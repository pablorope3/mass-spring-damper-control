"""Free-response simulation of a mass-spring-damper system.

Solves  m*x'' + c*x' + k*x = F(t)  in state-space form and compares the
numerical result with the analytical solution (underdamped case).
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

# ---------------------------------------------------------------------------
# Parameters (see docs/model.md)
# ---------------------------------------------------------------------------
m = 1.0   # mass [kg]
k = 10.0  # spring stiffness [N/m]
c = 1.0   # damping coefficient [Ns/m]

x0 = 1.0  # initial displacement [m]
v0 = 0.0  # initial velocity [m/s]

t_end = 10.0  # simulation time [s]


def force(t):
    """External force F(t) [N]. Zero for the free response."""
    return 0.0


# ---------------------------------------------------------------------------
# State-space model
# ---------------------------------------------------------------------------
A = np.array([[0.0, 1.0],
              [-k / m, -c / m]])
B = np.array([0.0, 1.0 / m])


def dynamics(t, state):
    """Right-hand side of  x_dot = A x + B u."""
    return A @ state + B * force(t)


# ---------------------------------------------------------------------------
# Numerical solution
# ---------------------------------------------------------------------------
t_eval = np.linspace(0.0, t_end, 1000)
sol = solve_ivp(
    dynamics,
    t_span=(0.0, t_end),
    y0=[x0, v0],
    t_eval=t_eval,
    rtol=1e-9,
    atol=1e-12,
)
x_num = sol.y[0]
v_num = sol.y[1]

# ---------------------------------------------------------------------------
# Analytical solution (underdamped, F(t) = 0)
# ---------------------------------------------------------------------------
wn = np.sqrt(k / m)
zeta = c / (2.0 * np.sqrt(k * m))
wd = wn * np.sqrt(1.0 - zeta**2)

x_ana = np.exp(-zeta * wn * t_eval) * (
    x0 * np.cos(wd * t_eval)
    + (v0 + zeta * wn * x0) / wd * np.sin(wd * t_eval)
)

# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
max_error = np.max(np.abs(x_num - x_ana))

print(f"Natural frequency  wn   = {wn:.4f} rad/s")
print(f"Damping ratio      zeta = {zeta:.4f}")
print(f"Damped frequency   wd   = {wd:.4f} rad/s")
print(f"Max |numerical - analytical| = {max_error:.2e} m")

# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

ax1.plot(t_eval, x_num, label="Numerical (solve_ivp)", linewidth=2)
ax1.plot(t_eval, x_ana, "--", label="Analytical", linewidth=2)
ax1.set_ylabel("Displacement x [m]")
ax1.set_title("Mass-spring-damper: free response")
ax1.grid(True)
ax1.legend()

ax2.plot(t_eval, v_num, color="tab:green", linewidth=2)
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Velocity [m/s]")
ax2.grid(True)

fig.tight_layout()

output_dir = Path(__file__).resolve().parent.parent / "results"
output_dir.mkdir(exist_ok=True)
fig.savefig(output_dir / "free_response.png", dpi=150)

plt.show()
