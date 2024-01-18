clc; clear;
% system parameters
M = 20;
l = 0.4;
Jyy = 0.6;
COM = [0,0,0];
g = 9.8;
b = 0.0;

% system state space function
A = [0, 1, 0, 0;
    M*g*l/Jyy, 0, 0, 0;
    0, 0, 0, 1;
    0, 0, 0, b/M];
B = [0; l/Jyy; 0; 1/M];

% LQR constrains
Q = diag([1, .001, 1, .01]);
R = .01*eye(1);

% solve state feedback gain matrix
K = lqr(A, B, Q, R);

% set model initial condition
initialAngle = deg2rad(20);