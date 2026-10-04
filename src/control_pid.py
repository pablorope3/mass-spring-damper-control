"""Closed-loop step response of a mass-spring-damper with P, PD and PID control.

Plant:        m*x'' + c*x' + k*x = u
Control laws: P   : u = Kp*e
              PD  : u = Kp*e - Kd*x'
              PID : u = Kp*e + Ki*integral(e) - Kd*x'
with e = r - x. The derivative acts on the measured velocity (not on the error),
which is equivalent for a constant reference and avoids a spike when r changes.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

# ---------------------------------------------------------------------------
# Plant parameters (see docs/mathematical_model.md)
# ---------------------------------------------------------------------------
m = 1.0   # mass [kg]
k = 10.0  # spring stiffness [N/m]
c = 1.0   # damping coefficient [Ns/m]

# ---------------------------------------------------------------------------
# Simulation settings
# ---------------------------------------------------------------------------
r = 1.0       # step reference [m]
x0 = 0.0      # initial displacement [m]
v0 = 0.0      # initial velocity [m/s]
t_end = 10.0  # simulation time [s]
t_eval = np.linspace(0.0, t_end, 2000)

# Controller gains: (Kp [N/m], Ki [N/(m*s)], Kd [Ns/m])
CONTROLLERS = {
    "P":   {"Kp": 20.0, "Ki": 0.0,  "Kd": 0.0},
    "PD":  {"Kp": 20.0, "Ki": 0.0,  "Kd": 8.0},
    "PID": {"Kp": 20.0, "Ki": 40.0, "Kd": 8.0},
}


def control_law(x, v, xi, gains):
    """Control force u. `xi` is the integral of the error."""
    e = r - x
    return gains["Kp"] * e + gains["Ki"] * xi - gains["Kd"] * v


def closed_loop(t, state, gains):
    """State = [x, v, xi]  with  xi_dot = e = r - x."""
    x, v, xi = state
    u = control_law(x, v, xi, gains)
    return [v, (u - c * v - k * x) / m, r - x]


def simulate(gains):
    sol = solve_ivp(
        closed_loop,
        t_span=(0.0, t_end),
        y0=[x0, v0, 0.0],
        t_eval=t_eval,
        args=(gains,),
        rtol=1e-9,
        atol=1e-12,
    )
    x, v, xi = sol.y
    u = control_law(x, v, xi, gains)
    return x, u


def metrics(x, u):
    """Overshoot [%], 2% settling time [s], steady-state error [m], max |u| [N]."""
    overshoot = max(0.0, (x.max() - r) / r * 100.0)

    outside = np.abs(x - r) > 0.02 * abs(r)
    if outside[-1]:
        settling = np.nan  # never stays within the 2% band
    elif outside.any():
        settling = t_eval[np.where(outside)[0][-1] + 1]
    else:
        settling = 0.0

    ss_error = r - x[-1]
    return overshoot, settling, ss_error, np.max(np.abs(u))


# ---------------------------------------------------------------------------
# Run all controllers
# ---------------------------------------------------------------------------
results = {}
for name, gains in CONTROLLERS.items():
    x, u = simulate(gains)
    results[name] = (x, u, metrics(x, u))

print(f"{'Ctrl':<5} {'Overshoot [%]':>14} {'Settling [s]':>13} "
      f"{'SS error [m]':>13} {'max|u| [N]':>11}")
for name, (_, _, (os_, ts, ess, umax)) in results.items():
    ts_txt = f"{ts:.2f}" if not np.isnan(ts) else "never"
    print(f"{name:<5} {os_:>14.1f} {ts_txt:>13} {ess:>13.4f} {umax:>11.1f}")

# Theoretical steady-state error of the P controller: r*k/(k+Kp)
Kp = CONTROLLERS["P"]["Kp"]
print(f"\nP controller, theoretical SS error: {r * k / (k + Kp):.4f} m")

# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

for name, (x, u, _) in results.items():
    ax1.plot(t_eval, x, label=name, linewidth=2)
    ax2.plot(t_eval, u, label=name, linewidth=2)

ax1.axhline(r, color="k", linestyle="--", linewidth=1, label="Reference")
ax1.set_ylabel("Displacement x [m]")
ax1.set_title("Step response: P vs PD vs PID")
ax1.grid(True)
ax1.legend()

ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Control force u [N]")
ax2.grid(True)
ax2.legend()

fig.tight_layout()

output_dir = Path(__file__).resolve().parent.parent / "results"
output_dir.mkdir(exist_ok=True)
fig.savefig(output_dir / "controllers_comparison.png", dpi=150)

plt.show()
