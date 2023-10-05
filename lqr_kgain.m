clc; clear;
M = 10;
g = 9.80665;
Jyy = 0.20983;
l = 0.25;

Q_gain = 100;
R_gain = .1;

A = [0, 1; M*g*l/Jyy, 0];
B = [0; l/Jyy];
Q = [Q_gain, 0; 0, Q_gain];
R = R_gain;

K = lqr(A, B, Q, R);
display(K);