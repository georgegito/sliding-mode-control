close all;
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
set(0,'defaulttextInterpreter','latex');
set(groot, 'defaultFigureUnits', 'points', 'defaultFigurePosition', [0 0 700*1.3 300*1.3]);
set(0, 'DefaultLineLineWidth', 0.8);

q1 = x(:, 1);
q2 = x(:, 3);
q1_dot = x(:, 2);
q2_dot = x(:, 4);
 
qd1 = pi/2;
qd2 = -pi/3;

% system states
figure(1);
plot(t, q1);
hold on;
plot(t, q2);
hold on;
fplot(qd1, t_span, '--');
hold on;
fplot(qd2, t_span, '--');
legend('$q_1$', '$q_2$', '$q_{d1}$', '$q_{d2}$');
xlabel('time ($s$)');
ylabel('rotation angle ($rad$)')
grid on;

figure(2);
plot(t, q1_dot);
hold on;
plot(t, q2_dot);
hold on;
fplot(0, t_span, '--');
legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_{d1}$, $\dot{q}_{d2}$');
xlabel('time ($s$)');
ylabel('angular velocity ($rad / s$)')
grid on;

% phase plane of system states
figure(3);
plot(q1, q1_dot);
hold on;
plot(q2, q2_dot);
hold on;
scatter(pi/3, 0, 'filled');
hold on;
scatter(qd1, 0, 'filled');
hold on;
scatter(qd2, 0, 'filled');
legend('$q_1, \dot{q}_1$', '$q_2, \dot{q}_2$', '$(q_{01}, \dot{q}_{01}), (q_{02}, \dot{q}_{02})$', '$(q_{d1}, \dot{q}_{d1})$', '$(q_{d2}, \dot{q}_{d2})$');
xlabel('rotation angle ($rad$)');
ylabel('angular velocity ($rad / s$)')
grid on;


% phase plane of error
e1 = e([q1 q2], [qd1 qd2]);
e1 = e1(:, 1);

e1_dot = e_dot([q1_dot q2_dot], [0 0]);
e1_dot = e1_dot(:, 1);

e2 = e([q1 q2], [qd1 qd2]);
e2 = e2(:, 2);

e2_dot = e_dot([q1_dot q2_dot], [0 0]);
e2_dot = e2_dot(:, 2);

figure(4);
plot(e1, e1_dot); 
hold on; 
plot(e2, e2_dot);
hold on;
fplot(@(x) -L(1, 1) * x, '--');
hold on;
scatter(e1(1), e1_dot(1), 'filled');
hold on;
scatter(e2(1), e2_dot(1), 'filled');
xlim([-1, 2.5]);
ylim([-15 5]);
legend('$s_1 (\dot{e}, e)$', '$s_2 (\dot{e}, e)$', '$s_1=0, s_2 = 0$', '$s_1(t=0)$', '$s_2(t=0)$');
xlabel('rotation angle error ($rad$)');
ylabel('angular velocity error ($rad / s$)')
grid on;

% error
e1 = zeros(size(t));
e2 = zeros(size(t));

for i = 1: length(t)
    e1(i) = q1(i) - qd1;
    e2(i) = q2(i) - qd2;
end

figure(5);
plot(t, e1);
hold on;
plot(t, e2);
hold on;
fplot(0, t_span);
legend('$e_1$', '$e_2$');
xlabel('time ($s$)');
ylabel('rotation angle error($rad$)')
grid on;


u_all = zeros(length(t), 2);
s_all = zeros(length(t), 2);
sat_all = zeros(length(t), 2);
for i = 1: length(t)
    s_all(i, :) = s([q1(i); q2(i)], [q1_dot(i); q2_dot(i)], [qd1; qd2], [0; 0])'; 
    sat_all(i, :) = [sat(s_all(i, 1), e_sat); sat(s_all(i, 2), e_sat)];
    u_all(i, :) = u([q1(i); q2(i)], [q1_dot(i); q2_dot(i)], [qd1; qd2], [0; 0], [0; 0], s_all(i, :)');
end

% controller
figure(6);
plot(t, u_all);
legend('u1', 'u2');
xlabel('time ($s$)');
ylabel('input torque ($N m$)')
grid on;

% surface
figure(7);
plot(t, s_all);
legend('$s_1(\dot{q}, q)$', '$s_2(\dot{q}, q)$');
xlabel('time ($s$)');
grid on;

% sat
figure(8);
plot(t, sat_all, 'linewidth', 0.5);
legend('$sat(s_1, 0.05)$', '$sat(s_2, 0.05)$');
xlabel('time ($s$)');
grid on;

