# LQR控制器仿真



## Pre-requirements

* Simulink 10.0 (R2019b) or later
* Simscape
* Control System Toolbox



## 倒立摆模型

```math
\begin{align*}
& x = Ax + Bu \\
& y = Cx
\end{align*}
```

对于理想的简单一阶倒立摆系统，我们可以计算其动力学方程，
```math
J\ddot{\theta} = Mgl\sin{\theta} + Fl\cos{\theta}
```
其中

* $J$ 是绕转动轴方向的转动惯量
* $M$ 是倒立摆的质量
* $g$ 是重力常数
* $l$ 是重心到转动轴的距离
* $\theta$ 是倒立摆的倾角

在小角度变化范围内，该系统可以被近似线性化表示，
```math
J\ddot{\theta} = Mgl\theta + Fl
```
选取状态变量，
```math
x = \left[\begin{matrix}
\theta \\
\dot{\theta}
\end{matrix}\right]
```
系统输入为，
```math
u = F
```


状态系统方程为，
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
将A, B矩阵输入lqr函数，定义损失约束矩阵$`Q = \left[\begin{matrix} q & 0 \\ 0 & q\end{matrix}\right]`$和$`R = \left[ r \right]`$,用MATLAB计算状态反馈矩阵K的数值，`K = lqr(A, B, Q, R)`，建立输出到输入的映射$`u = -Kx`$，即可实现LQR的状态闭环控制

