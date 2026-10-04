"""Comparison of P, PD, PID and LQR controllers on the mass-spring-damper.

Two scenarios are simulated with the SAME controllers (designed on the nominal model):
    1. Nominal : the real plant matches the model.
    2. Perturbed: the real stiffness is 20 % higher than the model and a constant
                  disturbance force acts on the mass from t = T_DIST on.

This script is self-contained (it does not import the other scripts, which run
a simulation when imported).
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp, trapezoid
from scipy.linalg import solve_continuous_are

# ---------------------------------------------------------------------------
# Nominal plant (the model used to design the controllers)
# ---------------------------------------------------------------------------
m = 1.0   # mass [kg]
k = 10.0  # spring stiffness [N/m]
c = 1.0   # damping coefficient [Ns/m]

A = np.array([[0.0, 1.0], [-k / m, -c / m]])
B = np.array([[0.0], [1.0 / m]])

# ---------------------------------------------------------------------------
# Simulation settings
# ---------------------------------------------------------------------------
r = 1.0       # step reference [m]
t_end = 12.0  # simulation time [s]
t_eval = np.linspace(0.0, t_end, 3000)

# Perturbed scenario
K_REAL_FACTOR = 1.2  # real stiffness = 1.2 * k
F_DIST = 3.0         # constant disturbance force [N]
T_DIST = 6.0         # time at which the disturbance appears [s]

# ---------------------------------------------------------------------------
# Controller design
# ---------------------------------------------------------------------------
GAINS = {
    "P":   {"Kp": 20.0, "Ki": 0.0,  "Kd": 0.0},
    "PD":  {"Kp": 20.0, "Ki": 0.0,  "Kd": 8.0},
    "PID": {"Kp": 20.0, "Ki": 40.0, "Kd": 8.0},
}

Q = np.diag([100.0, 1.0])
R = np.array([[1.0]])
P_are = solve_continuous_are(A, B, Q, R)
K_lqr = np.linalg.solve(R, B.T @ P_are).flatten()  # [K1, K2]
N_lqr = k + K_lqr[0]  # feedforward, designed with the NOMINAL stiffness


def make_controller(name):
    """Return u(x, v, xi) for the given controller."""
    if name == "LQR":
        return lambda x, v, xi: -K_lqr[0] * x - K_lqr[1] * v + N_lqr * r
    g = GAINS[name]
    return lambda x, v, xi: g["Kp"] * (r - x) + g["Ki"] * xi - g["Kd"] * v


NAMES = ["P", "PD", "PID", "LQR"]


def simulate(name, k_real, f_dist):
    """Closed-loop simulation. State = [x, v, xi] with xi_dot = r - x."""
    control = make_controller(name)

    def dynamics(t, s):
        x, v, xi = s
        u = control(x, v, xi)
        f_d = f_dist if t >= T_DIST else 0.0
        return [v, (u - c * v - k_real * x + f_d) / m, r - x]

    sol = solve_ivp(dynamics, (0.0, t_end), [0.0, 0.0, 0.0], t_eval=t_eval,
                    max_step=0.01, rtol=1e-8, atol=1e-10)
    x, v, xi = sol.y
    u = np.array([control(*s) for s in sol.y.T])
    return x, u


def metrics(x, u, t_window):
    """Metrics computed on t <= t_window."""
    mask = t_eval <= t_window
    xs, us, ts = x[mask], u[mask], t_eval[mask]

    overshoot = max(0.0, (xs.max() - r) / r * 100.0)

    outside = np.abs(xs - r) > 0.02 * abs(r)
    if outside[-1]:
        settling = np.nan
    elif outside.any():
        settling = ts[np.where(outside)[0][-1] + 1]
    else:
        settling = 0.0

    ss_error = r - xs[-1]
    ise = trapezoid((r - xs) ** 2, ts)   # integral of squared error
    effort = trapezoid(us ** 2, ts)      # integral of squared control force
    return overshoot, settling, ss_error, ise, effort, np.max(np.abs(us))


def fmt_settling(ts):
    return f"{ts:.2f}" if not np.isnan(ts) else "never"


# ---------------------------------------------------------------------------
# Run both scenarios
# ---------------------------------------------------------------------------
nominal = {n: simulate(n, k, 0.0) for n in NAMES}
perturbed = {n: simulate(n, K_REAL_FACTOR * k, F_DIST) for n in NAMES}

print(f"LQR gain K = [{K_lqr[0]:.3f}, {K_lqr[1]:.3f}], N = {N_lqr:.3f}\n")

print("=== Scenario 1: nominal plant (metrics over the whole simulation) ===")
print(f"{'Ctrl':<5} {'Overshoot[%]':>13} {'Settling[s]':>12} {'SS err[m]':>10} "
      f"{'ISE':>8} {'Effort':>9} {'max|u|[N]':>10}")
for n in NAMES:
    os_, ts, ess, ise, eff, umax = metrics(*nominal[n], t_end)
    print(f"{n:<5} {os_:>13.1f} {fmt_settling(ts):>12} {ess:>10.4f} "
          f"{ise:>8.3f} {eff:>9.1f} {umax:>10.1f}")

print(f"\n=== Scenario 2: k +{(K_REAL_FACTOR - 1) * 100:.0f} %, "
      f"disturbance {F_DIST} N at t = {T_DIST} s ===")
print(f"{'Ctrl':<5} {'Error before dist.[m]':>22} {'Error at end[m]':>16}")
for n in NAMES:
    x, _ = perturbed[n]
    before = r - x[np.searchsorted(t_eval, T_DIST) - 1]
    after = r - x[-1]
    print(f"{n:<5} {before:>22.4f} {after:>16.4f}")

# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
scenarios = [
    (nominal, "Scenario 1: nominal plant"),
    (perturbed, f"Scenario 2: k +{(K_REAL_FACTOR - 1) * 100:.0f} % and "
                f"{F_DIST:g} N disturbance at t = {T_DIST:g} s"),
]

for col, (data, title) in enumerate(scenarios):
    ax_x, ax_u = axes[0, col], axes[1, col]
    for n in NAMES:
        x, u = data[n]
        ax_x.plot(t_eval, x, label=n, linewidth=2)
        ax_u.plot(t_eval, u, label=n, linewidth=2)
    ax_x.axhline(r, color="k", linestyle="--", linewidth=1)
    ax_x.set_title(title)
    ax_x.grid(True)
    ax_u.set_xlabel("Time [s]")
    ax_u.grid(True)
    if col == 1:
        for ax in (ax_x, ax_u):
            ax.axvline(T_DIST, color="gray", linestyle=":", linewidth=1)

axes[0, 0].set_ylabel("Displacement x [m]")
axes[1, 0].set_ylabel("Control force u [N]")
axes[0, 0].legend()

fig.tight_layout()

output_dir = Path(__file__).resolve().parent.parent / "results"
output_dir.mkdir(exist_ok=True)
fig.savefig(output_dir / "controllers_summary.png", dpi=150)

plt.show()
