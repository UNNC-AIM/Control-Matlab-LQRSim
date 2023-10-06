# LQR Controller Design



## Pre-requirements

* Simulink 10.0 (R2019b) or later
* Simscape
* Control System Toolbox



## Inverted Pendulum Model

```math
\begin{align*}
& x = Ax + Bu \\
& y = Cx
\end{align*}
```

For idealized inverted pendulum system, we can calculate the dynamic equation,

```math
J\ddot{\theta} = Mgl\sin{\theta} + Fl\cos{\theta}
```

Where

* $J$ is the moment of inertia
* $M$ is the mass of the inverted pendulum
* $g$ is the gravity constant
* $l$ is the distance between center of mass and the revolving joint
* $\theta$ is the tilting angle of the inverted pendulum

In a small angle range, the system can be approximated to a linear system，

```math
J\ddot{\theta} = Mgl\theta + Fl
```

We can choose the state $x$，

```math
x = \left[\begin{matrix}
\theta \\
\dot{\theta}
\end{matrix}\right]
```

The input of the system is，

```math
u = F
```


The state space equation for this system is，

```math
\begin{align*}
& \dot{x} = \left[\begin{matrix}
0 & 1 \\
\frac{Mgl}{J} & 0
\end{matrix}\right]x + \left[\begin{matrix}
0 \\
\frac{l}{J}
\end{matrix}\right]u \\
& y = \left[\begin{matrix}
1 & 0 \\
0 & 1
\end{matrix}\right]x
\end{align*}
```

Input Matrix A, B to lqr function, define the constrain matrix for state and input, $`Q = \left[\begin{matrix} q & 0 \\ 0 & q\end{matrix}\right]`$ and $`R = \left[ r \right]`$,using MATLAB to calculate the value of state feedback matrix，`K = lqr(A, B, Q, R)`，build the mapping from output to input, $`u = -Kx`$, the LQR control can be realized.

