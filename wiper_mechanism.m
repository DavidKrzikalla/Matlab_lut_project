% Slider crank kinematic analysis
close all
clc

%Geometrical settings
L2 = 0.03;
L3 = 0.14;
L4 = 0.197;
xD = 0.087;
yD = 0.10624;

%Inital rotational speed
omega0 = 4;
tend = 2; %final time of analysis
dt = 0.001; % time step

%Initial coordinates
alpha0 = 120;
xG2 = -L2/2*cosd(180-alpha0);
yG2 = L2/2*sind(180-alpha0);
xB = 2*xG2;
yB = 2*yG2;
beta0 = 18.29;
xC = xB + L3*cosd(beta0);
yC = yB + L3*sind(beta0);
xG3 = xB + L3/2*cosd(beta0);
yG3 = yB + L3/2*sind(beta0);
gamma0 = 180 + atand((yD-yC)/(xD-xC));
xG4 = xC + L4/2*cosd(gamma0);
yG4 = yC + L4/2*sind(gamma0);

%Check of shape
figure
scatter(xD,yD)
hold on
axis equal
scatter(xB,yB)
scatter(xC,yC)
scatter(xG2,yG2)
scatter(xG3,yG3)
scatter(xG4,yG4)
scatter(0,0)
plot([0,xB,xC,xG4],[0,yB,yC,yG4],'-o')
plot(0, 0, '*', 'LineWidth', 2)
title('Mechanism scheme')

%% Coordinates
% ground
q1 = [0; 0; 0];

% crank gravity center
q2 = [xG2
    yG2
    pi/180*(alpha0)];

% link gravity center
q3 = [xG3
    yG3
    pi/180*(beta0)];

% wiper gravity center
q4 = [xG4
    yG4
    pi/180*(gamma0)];

q_0 = [q1; q2; q3; q4]; % initial coordinates
%% Revolute joints
% 1 connects ground 1 and crank
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [-L2/2; 0];

% 2 connects crank and link
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [L2/2; 0];
revolute(2).s_j = [-L3/2; 0];

% 3 connects link and wiper
revolute(3).i = 3;
revolute(3).j = 4;
revolute(3).s_i = [L3/2; 0];
revolute(3).s_j = [-L4/2; 0];

% 4 connects ground 1 and wiper
revolute(4).i = 4;
revolute(4).j = 1;
revolute(4).s_i = [-sqrt((xG4-xD)^2+(yG4-yD)^2); 0];
revolute(4).s_j = [xD; yD];

% % % Check revolute joint constraints
% r = revolute(4);
% C_r_i = revolute_joint(r.i, r.j, r.s_i, r.s_j, q_0);
%% Simple constraints

% Three simple joints to fix the ground origin
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;

% % % check simple constraints
%  for s = simple
%      C_s_i = simple_joint(s.i, s.k, s.c_k, q_0);
%  end
%% Add some driving constraints
driving.i = 2;
driving.k = 3;
driving.d_k = @(t)  pi/180*(alpha0) - omega0 * t;
driving.d_k_t = @(t) -omega0;
driving.d_k_tt = @(t) 0;

% % % Verify
% d = driving(1);
% C_d_i = driving_joint(d.i, d.k, d.d_k, 0, q_0)

% %% Verify constraint function
%clc;
C = constraint(revolute, simple, driving, 0, q_0);

%% Jacobian of our constraints

Cq = constraint_dq(revolute, simple, driving, 0, q_0);
%% Verify Ct
Ct = constraint_dt(revolute, simple, driving, 0, q_0);
%% Solve constraint equation using NR for position and velocity
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
Ctt_fun = @(t, q, dq) constraint_dtt(revolute, simple, driving, t, q, dq);
[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun, Ctt_fun, tend, q_0,dt);

%% Position plot
figure
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 10), Q(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
legend('body2','body 3','wiper')
title('Position')
xlabel('X[m]')
ylabel('Y[m]')
%% Velocity plot
figure
plot(QP(:, 4), QP(:, 5), ...
    QP(:, 7), QP(:, 8), ...
    QP(:, 10), QP(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
legend('body2','body 3','wiper')
title('Velocity')
xlabel('vx[m/s]')
ylabel('vy[m/s]')
%% Acceleration plot
figure
plot(QPP(:, 4), QPP(:, 5), ...
    QPP(:, 7), QPP(:, 8), ...
    QPP(:, 10), QPP(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
legend('body2','body 3','wiper')
title('Acceleration')
xlabel('ax[m/s^2]')
ylabel('ay[m/s^2]')