clear; clc;

% system parameters
M = 20;
l = 0.4;
Jyy = 0.6;
COM = [0,0,0];
g = 9.8;
b = 0.0;

% Back-stepping gains
k1 = 10;
k2 = 2;

% set model initial condition
initialAngle = deg2rad(20);