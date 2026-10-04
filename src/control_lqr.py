"""LQR control of a mass-spring-damper system.

Plant (state-space):  x_dot = A x + B u,   x = [position, velocity]
Control law:          u = -K x + N r
    K : LQR gain, from the algebraic Riccati equation  A'P + PA - P B R^-1 B' P + Q = 0
        K = R^-1 B' P
    N : reference feedforward gain so that the steady-state position equals r.
        For this plant N = k + K[0] (it compensates the spring force at x = r).

The script also shows the effect of the weight R (cheap vs expensive control).
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

# ---------------------------------------------------------------------------
# Plant parameters (see docs/mathematical_model.md)
# ---------------------------------------------------------------------------
m = 1.0   # mass [kg]
k = 10.0  # spring stiffness [N/m]
c = 1.0   # damping coefficient [Ns/m]

A = np.array([[0.0, 1.0],
              [-k / m, -c / m]])
B = np.array([[0.0],
              [1.0 / m]])

# ---------------------------------------------------------------------------
# Simulation settings
# ---------------------------------------------------------------------------
r = 1.0       # step reference [m]
t_end = 10.0  # simulation time [s]
t_eval = np.linspace(0.0, t_end, 2000)

# ---------------------------------------------------------------------------
# LQR weights
# ---------------------------------------------------------------------------
Q = np.diag([100.0, 1.0])  # penalty on [position, velocity]
R_MAIN = 1.0               # penalty on control effort (reference design)
R_SWEEP = [0.01, 0.1, 1.0, 10.0]


def lqr_gain(Q, R):
    """Solve the continuous-time algebraic Riccati equation and return K, P."""
    R = np.atleast_2d(R)
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)  # K = R^-1 B' P
    return K, P


def simulate(K):
    """Closed-loop step response with u = -K x + N r."""
    N = k + K[0, 0]

    def dynamics(t, state):
        u = (-K @ state + N * r).item()
        return A @ state + B.flatten() * u

    sol = solve_ivp(dynamics, (0.0, t_end), [0.0, 0.0], t_eval=t_eval,
                    rtol=1e-9, atol=1e-12)
    x = sol.y[0]
    u = -(K @ sol.y).flatten() + N * r
    return x, u


def metrics(x, u):
    """Overshoot [%], 2% settling time [s], steady-state error [m], max |u| [N]."""
    overshoot = max(0.0, (x.max() - r) / r * 100.0)
    outside = np.abs(x - r) > 0.02 * abs(r)
    if outside[-1]:
        settling = np.nan
    elif outside.any():
        settling = t_eval[np.where(outside)[0][-1] + 1]
    else:
        settling = 0.0
    return overshoot, settling, r - x[-1], np.max(np.abs(u))


# ---------------------------------------------------------------------------
# Reference design
# ---------------------------------------------------------------------------
K, P = lqr_gain(Q, R_MAIN)
poles = np.linalg.eigvals(A - B @ K)

print(f"Q = diag({Q[0, 0]:g}, {Q[1, 1]:g}), R = {R_MAIN}")
print(f"LQR gain K = [{K[0, 0]:.4f}, {K[0, 1]:.4f}]")
print(f"Open-loop poles   : {np.round(np.linalg.eigvals(A), 3)}")
print(f"Closed-loop poles : {np.round(poles, 3)}  (all Re < 0: stable)")

# ---------------------------------------------------------------------------
# Effect of R
# ---------------------------------------------------------------------------
results = {}
for R in R_SWEEP:
    K_i, _ = lqr_gain(Q, R)
    x, u = simulate(K_i)
    results[R] = (K_i, x, u, metrics(x, u))

print(f"\n{'R':>6} {'K1':>8} {'K2':>8} {'Overshoot [%]':>14} "
      f"{'Settling [s]':>13} {'SS error [m]':>13} {'max|u| [N]':>11}")
for R, (K_i, _, _, (os_, ts, ess, umax)) in results.items():
    ts_txt = f"{ts:.2f}" if not np.isnan(ts) else "never"
    print(f"{R:>6} {K_i[0, 0]:>8.2f} {K_i[0, 1]:>8.2f} {os_:>14.1f} "
          f"{ts_txt:>13} {ess:>13.5f} {umax:>11.1f}")

# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

for R, (_, x, u, _) in results.items():
    ax1.plot(t_eval, x, label=f"R = {R}", linewidth=2)
    ax2.plot(t_eval, u, label=f"R = {R}", linewidth=2)

ax1.axhline(r, color="k", linestyle="--", linewidth=1, label="Reference")
ax1.set_ylabel("Displacement x [m]")
ax1.set_title("LQR step response for different control weights R")
ax1.grid(True)
ax1.legend()

ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Control force u [N]")
ax2.grid(True)
ax2.legend()

fig.tight_layout()

output_dir = Path(__file__).resolve().parent.parent / "results"
output_dir.mkdir(exist_ok=True)
fig.savefig(output_dir / "lqr_response.png", dpi=150)

plt.show()
