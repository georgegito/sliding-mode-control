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

qd1 = @(t) pi/4 + pi/6 * sin(0.2*pi*t);
qd2 = @(t) -pi/3 + pi/3 * cos(0.2*pi*t);
qd1_dot = @(t) pi^2 / 30 * cos(0.2*pi*t);
qd2_dot = @(t) -pi^2 / 15 * sin(0.2*pi*t);

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
fplot(qd1_dot, t_span, '--');
hold on; 
fplot(qd2_dot, t_span, '--');
legend('$\dot{q}_1$', '$\dot{q}_2$', '$\dot{q}_{d1}$', '$\dot{q}_{d2}$');
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
plot(qd1(t), qd1_dot(t), '--r');
hold on;
plot(qd2(t), qd2_dot(t), '--g');
legend('$q_1, \dot{q}_1$', '$q_2, \dot{q}_2$', '$(q_{01}, \dot{q}_{01}), (q_{02}, \dot{q}_{02})$', '$q_{d1}, \dot{q}_{d1}$', '$q_{d2}, \dot{q}_{d2}$');
xlabel('rotation angle ($rad$)');
ylabel('angular velocity ($rad / s$)')
grid on;


% phase plane of error
e1 = e([q1 q2], [qd1(t) qd2(t)]);
e1 = e1(:, 1);

e1_dot = e_dot([q1_dot q2_dot], [qd1_dot(t) qd2_dot(t)]);
e1_dot = e1_dot(:, 1);

e2 = e([q1 q2], [qd1(t) qd2(t)]);
e2 = e2(:, 2);

e2_dot = e_dot([q1_dot q2_dot], [qd1_dot(t) qd2_dot(t)]);
e2_dot = e2_dot(:, 2);

figure(4);
plot(e1, e1_dot); 
hold on; 
plot(e2, e2_dot);
hold on;
fplot(@(x) -L(1, 1) * x, '--');
hold on;
fplot(@(x) -L(2, 2) * x, '--');
scatter(e1(1), e1_dot(1), 'filled');
hold on;
scatter(e2(1), e2_dot(1), 'filled');
xlim([-1, 2.5]);
ylim([-15 5]);
legend('$s_1 (\dot{e}, e)$', '$s_2 (\dot{e}, e)$', '$s_1=0$', '$s_2 = 0$', '$s_1(t=0)$', '$s_2(t=0)$');
xlabel('rotation angle error ($rad$)');
ylabel('angular velocity error ($rad / s$)')
grid on;


% error
e1 = zeros(size(t));
e2 = zeros(size(t));

e1(:) = q1(:) - qd1(t(:));
e2(:) = q2(:) - qd2(t(:));

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


% controller
u_all = zeros(length(t), 2);
s_all = zeros(length(t), 2);
sat_all = zeros(length(t), 2);
for i = 1: length(t)
    s_all(i, :) = s([q1(i); q2(i)], [q1_dot(i); q2_dot(i)], [qd1(t(i)); qd2(t(i))], [qd1_dot(t(i)); qd2_dot(t(i))])'; 
    sat_all(i, :) = [sat(s_all(i, 1), e_sat); sat(s_all(i, 2), e_sat)];
    u_all(i, :) = u([q1(i); q2(i)], [q1_dot(i); q2_dot(i)], qd(t(i)), qd_dot(t(i)), qd_ddot(t(i)), s_all(i, :)');
end

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