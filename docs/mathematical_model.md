# Mathematical Model

## 1. System Description

Te system consists of a mass connected to a spring and a damper. 
The mass is constrained to move along one dimension.

The system has one degree of freedom, described by the displacement:

$$
x(t)
$$

The system parameters are:

- $m$: mass [kg]
- $k$: spring stiffness [N/m]
- $c$: damping coefficient [Ns/m]
- $F(t)$: external force [N]

## 2. Physical Model

The equation of motion is obtained from Newton's second law:

$$
\sum F = m\ddot{x}
$$

The forces acting on the mass are:

### Spring Force

According to Hooke's law:

$$
F_k = -kx
$$

The negative sign indicates that the spring force opposes the displacement.

### Damping Force

The damping force is proportional to the velocity:

$$
F_c = -c\dot{x}
$$

The damping force always opposes the direction of motion.

### External Force

An external force $F(t)$ can be applied to the system.

## 3. Equation of Motion

Applying Newton's second law:

$$
m\ddot{x} = F(t) - c\dot{x} - kx
$$

Rearranging:

$$
\boxed{
m\ddot{x} + c\dot{x} + kx = F(t)
}
$$

This is the fundamental equation used to describe the dynamics of the system.

## 4. State-Space Representation

Define the state variables as:

$$
x_1 = x
$$

$$
x_2 = \dot{x}
$$

Therefore:

$$
\dot{x}_1 = x_2
$$

From the equation of motion:

$$
\ddot{x}
=
\frac{F(t)-c\dot{x}-kx}{m}
$$

Therefore:

$$
\dot{x}_2 =
-\frac{k}{m}x_1
-\frac{c}{m}x_2
+\frac{1}{m}F(t)
$$

The state-space representation can be written as:

$$
\dot{\mathbf{x}} = A\mathbf{x}+B\mathbf{u}
$$

where:

$$
\mathbf{x}
=
\begin{bmatrix}
x \\
\dot{x}
\end{bmatrix}
$$

and:

$$
A =
\begin{bmatrix}
0 & 1 \\
-\frac{k}{m} & -\frac{c}{m}
\end{bmatrix}
$$

$$
B =
\begin{bmatrix}
0 \\
\frac{1}{m}
\end{bmatrix}
$$

## 5. System Parameters

The initial simulation will use the following parameters:

| Parameter | Symbol | Value | Unit |
|---|---:|---:|---|
| Mass | $m$ | 1 | kg |
| Spring stiffness | $k$ | 10 | N/m |
| Damping coefficient | $c$ | 1 | Ns/m |

## 6. Initial Conditions

The system will initially be displaced from its equilibrium position and released:

$$
x(0) = 1\ m
$$

$$
\dot{x}(0) = 0\ m/s
$$

No external force is applied during the initial free-response simulation:

$$
F(t) = 0
$$

## 7. System Characteristics

The natural frequency is:

$$
\omega_n = \sqrt{\frac{k}{m}}
$$

For the selected parameters:

$$
\omega_n = \sqrt{10}
\approx 3.16\ rad/s
$$

The damping ratio is:

$$
\zeta =
\frac{c}{2\sqrt{km}}
$$

Therefore:

$$
\zeta =
\frac{1}{2\sqrt{10}}
\approx 0.158
$$

Since:

$$
0 < \zeta < 1
$$

the system is underdamped.

Therefore, the free response is expected to exhibit oscillations whose amplitude decreases over time.