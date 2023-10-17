clc; clear;
%% LQR
M = 20;
l = 0.30;
Jyy = 0.65;
g = 9.80665;
b = 0;

A = [0, 1, 0, 0;
    M*g*l/Jyy, 0, 0, 0;
    0, 0, 0, 1;
    0, 0, 0, b/M];
B = [0; l/Jyy; 0; 1/M];
Q = diag([.01, .01, 1, .01]);
R = .01;

K = lqr(A, B, Q, R);
% display(K);
% fprintf("Eigen Vector for no controller applied system:\n");
% display(eig(A));
% fprintf("Eigen Vector for LQR controlled system:\n");
% display(eig(A-B*K));

%% Sliding Mode
G = (B'*B)^-1*B';
rou = 1;