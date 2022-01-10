clear;
close all;
format shortG;
addpath("./lib");

%% controller parameters
L = [5 0; 0 5];
C_const = 0.5;

%% load data
sim1_data;

%% simulation
x0 = [pi/3 pi/3; 0 0];
t_span = [0 1.5];
qd = [pi/2; -pi/3];

tic;
[t, x] = ode45(@(t, x) dynamics1(t, x, u, H, C, g, qd, s), t_span, x0);
toc

%% plots
sim1_plots;