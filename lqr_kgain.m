clc; clear;
%% Real System Parameters
M = 28.52;
l = 0.3658;
Jyy = 0.654;
COM = [0.12,0,0];
g = 9.80665;
b = 0.12;

initialAngle = deg2rad(20);

%% LQR
A = [0, 1, 0, 0;
    M*g*l/Jyy, 0, 0, 0;
    0, 0, 0, 1;
    0, 0, 0, b/M];
B = [0; l/Jyy; 0; 1/M];
Q = diag([1, .001, 1, .01]);
R = .01*eye(1);

% u = -Kx
K = lqr(A, B, Q, R);
Kr = 1;

%% Sliding Mode
G = (B'*B)^-1*B';
rou = 1;

% s = Gx - Gx_0 - G\int_0^t(Ax + Bu)
% u = -Kx - rho(GB)^(-1) s/(||s|| + 1e-4)

%% Model Reference Adaptive Control

% Measured system parameters
M_ideal = 20;
l_ideal = 0.4;
Jyy_ideal = 0.6;
COM_ideal = [0,0,0];
g_ideal = 9.8;
b_ideal = 0.2;

A_est = [0, 1, 0, 0;
    M_ideal*g_ideal*l_ideal/Jyy_ideal, 0, 0, 0;
    0, 0, 0, 1;
    0, 0, 0, b_ideal/M_ideal];
B_est = [0; l_ideal/Jyy_ideal; 0; 1/M_ideal];

K_est = lqr(A_est, B_est, Q, R);

% The reference system model
Am = A_est - B_est*K_est;
Kr_est = 1;

%% Using LMI and convex optimization to solve LQR problem

% Coveriance matrix for initial state with 0 means
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

% u = -Kx
K_cvx = Y*X^(-1);

% System stability judgement
if eig(A-B*K_cvx) < 0
    disp('The system is stable')
else
    disp('The system is unstable')
end
