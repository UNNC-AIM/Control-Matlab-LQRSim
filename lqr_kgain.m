clc; clear;
%% Real System Implimentation
M = 28.52;
l = 0.3658;
Jyy = 0.654;
COM = [1.2,0,0];
g = 9.80665;
b = 0.12;

initialAngle = deg2rad(20);

%% Predicted System Implimentation
M_ideal = 20;
l_ideal = 0.3;
Jyy_ideal = 0.6;
COM_ideal = [0,0,0];
g_ideal = 9.8;
b_ideal = 0;

%% LQR
A = [0, 1, 0, 0;
    M*g*l/Jyy, 0, 0, 0;
    0, 0, 0, 1;
    0, 0, 0, b/M];
B = [0; l/Jyy; 0; 1/M];
Q = diag([.01, .01, 1, .01]);
R = .01;

K = lqr(A, B, Q, R);
Kr = 1;

%% Sliding Mode
G = (B'*B)^-1*B';
rou = 1;

%% Model Reference Adaptive Control
A_est = [0, 1, 0, 0;
    M_ideal*g_ideal*l_ideal/Jyy_ideal, 0, 0, 0;
    0, 0, 0, 1;
    0, 0, 0, b_ideal/M_ideal];
B_est = [0; l_ideal/Jyy_ideal; 0; 1/M_ideal];

K_est = lqr(A, B, Q, R);

Am = A_est - B_est*K_est;
Kr_est = 1;