clear; close all; clc;
% Real System Parameters
M = 28.52;
l = 0.3658;
Jyy = 0.654;
COM = [0.12,0,0];
g = 9.80665;
b = 0.12;

% state space function
A = [0, 1, 0, 0;
    M*g*l/Jyy, 0, 0, 0;
    0, 0, 0, 1;
    0, 0, 0, b/M];
B = [0; l/Jyy; 0; 1/M];

% LQR - non-convex method
Q = diag([1, .001, 1, .01]);
R = .01*eye(1);

K = lqr(A, B, Q, R);
Kr = 1;

% LQR - convex method
omega = eye(4);

cvx_begin sdp
variable X(4,4) symmetric
variable Z(1,1)
variable Y(1,4)

minimize(trace(Q^(1/2)*X*Q^(1/2)) + trace(Z))
A*X - B*Y + X*A' - Y'*B' + omega == 0;
[    Z,        R^(1/2)*Y;
 Y'*R^(1/2),       X     ] >=0;
cvx_end

K_cvx = Y*X^(-1);

% System stability judgement
if eig(A-B*K_cvx) < 0
    disp('The system is stable')
else
    disp('The system is unstable')
end