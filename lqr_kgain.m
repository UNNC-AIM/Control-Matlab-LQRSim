clc; clear;
%% LQR
M = 20;
l = 0.30;
% Jyy = 0.1515;
Jyy = 0.65;
g = 9.80665;
b = 0;

% Q_gain = 1;
R_gain = .01;

% A = [0, 1; M*g*l/Jyy, 0];
% B = [0; l/Jyy];
% Q = diag([Q_gain, Q_gain]);
% R = R_gain;

A = [0, 1, 0, 0;
    M*g*l/Jyy, 0, 0, 0;
    0, 0, 0, 1;
    0, 0, 0, b/M];
B = [0; l/Jyy; 0; 1/M];
Q = diag([.01, .01, 1, .01]);
R = R_gain;

K = lqr(A, B, Q, R);
display(K);
fprintf("Eigen Vector for no controller applied system:\n");
display(eig(A));
fprintf("Eigen Vector for LQR controlled system:\n");
display(eig(A-B*K));

%% Sliding Mode
e = 0.1;
k = 30;
c1 = 4;
c2 = 0.6;
c3 = 0.8;

G = [1, 1, 1, 1];

Tau = diag([1,1,1,1])-B*(G*B)^(-1)*G;
u = (G*B)^(-1)*G*A;
display(Tau*A)