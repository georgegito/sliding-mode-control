clear;
close all;
format shortG;
addpath("./lib");

%% controller parameters
L = [2 0; 0 5];
C_const = 10;

%% load data
sim2_data;

%% simulation
x0 = [pi/3 pi/3; 0 0];
t_span = [0 10];

qd = @(t) ...
      [pi/4 + pi/6 * sin(0.2*pi*t); ...
       -pi/3 + pi/3 * cos(0.2*pi*t)];
   
qd_dot = @(t) ...
               [pi^2 / 30 * cos(0.2*pi*t); ...
                -pi^2 / 15 * sin(0.2*pi*t)];
    
qd_ddot = @(t) ...
               [-pi^3 / 150 * sin(0.2*pi*t); ...
                -pi^3 / 75 * cos(0.2*pi*t)];
  
tic;
[t, x] = ode45(@(t, x) dynamics2(t, x, u, H, C, g, qd, qd_dot, qd_ddot, s), t_span, x0);
toc

%% plots
sim2_plots;