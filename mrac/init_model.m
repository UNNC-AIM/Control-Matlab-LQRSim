clc; clear;
% Actual system parameters
M = 28.52;
l = 0.3658;
Jyy = 0.654;
COM = [0.12,0,0];
g = 9.80665;
b = 0.12;

% Measured system parameters
M_ideal = 20;
l_ideal = 0.4;
Jyy_ideal = 0.6;
COM_ideal = [0,0,0];
g_ideal = 9.8;
b_ideal = 0.2;

% system state space function
A_est = [0, 1, 0, 0;
    M_ideal*g_ideal*l_ideal/Jyy_ideal, 0, 0, 0;
    0, 0, 0, 1;
    0, 0, 0, b_ideal/M_ideal];
B_est = [0; l_ideal/Jyy_ideal; 0; 1/M_ideal];

% LQR constrains
Q = diag([1, .001, 1, .01]);
R = .01*eye(1);

K_est = lqr(A_est, B_est, Q, R);

% The reference system model
Am = A_est - B_est*K_est;
Kr_est = 1;
Bm = B_est * Kr_est;

% set model initial condition
initialAngle = deg2rad(20);