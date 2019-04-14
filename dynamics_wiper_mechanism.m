%% Wiper mechanism solved for kinematics response on given force

clc

wiper_mechanism %load of geometry and neccessary variables
close all

tend =1; %final time of analysis
beta =2000; %stability coefficient

%% Bodies definition
% 3 bodies + 2 grounds (coded as one ground)

%ground: number 1
bodies(1).m = 1; % kg
bodies(1).Ic = 1; % kgm2
bodies(1).q = q1;

%crank: number 2
bodies(2).m = 0.5; %  kg
bodies(2).Ic = bodies(2).m * L2^2 / 12; % kgm2
bodies(2).q = q2;

%link 3: number 3
bodies(3).m = 1; % kg
bodies(3).Ic = bodies(3).m * L3^2 / 12; %  kgm2
bodies(3).q = q3;

%wiper: number 4
bodies(4).m = 1.5; %  kg
bodies(4).Ic = bodies(4).m * L4^2 / 12; %  kgm2
bodies(4).q = q4;

grav = [0; -9.81]; % gravitational acceleration

%% Get mass matrix

q0_dyn = system_coordinates(bodies); %Moving bodies coordinate system

M = mass_matrix(bodies);
%% Force vector building
sforce.f = [0;0];
sforce.i = 2; % 
sforce.u_i = [0;0];
sforce.t=-0.1;

%% Jacobian of our constraints
F_dyn = @(q) force_vector(grav, sforce, bodies, q);

C_fun_dyn = @(t, q) constraint_dyn(revolute, simple, t, q);
Cq_fun_dyn = @(t, q) constraint_dq_dyn(revolute, simple, t, q);
Ctt_fun_dyn = @(t, q, dq) constraint_dtt_dyn(revolute, simple, q, dq); %g function

g_hat = @(t, q, dq) Ctt_fun_dyn(t,q,dq)- beta^2*C_fun_dyn(t,q); %middle expression is neglected because Ct_fun is zero

%% Matrix building 
 
M_big = @(t,q) [M, Cq_fun_dyn(t,q)';
      Cq_fun_dyn(t,q), zeros(11,11)];
 
F_big = @(t,q,dq) [F_dyn(q) ; g_hat(t,q,dq)];

opts=odeset('AbsTol',1e-9,'RelTol',1e-8);
[T,U]=ode45(@(t, q)accODE(t, q, M, F_dyn, Cq_fun_dyn, g_hat),[0, tend],[q0_dyn; zeros(size(q0_dyn))],opts);

%% Plots
figure
plot(U(:, 4), U(:, 5),...
    U(:, 7), U(:, 8),...
    U(:, 10), U(:, 11),...
    0, 0, '*', ...
    xD, yD, '*','LineWidth', 1);
axis equal
legend('link 1', 'link 2', 'Wiper','Ground 1', 'Ground 2')
title('Position')
xlabel('x[m]')
ylabel('y[m]')

figure
plot(U(:, 16), U(:, 17),...
    U(:, 19), U(:, 20),...
    U(:, 22), U(:, 23),...
    0, 0, '*', ...
    xD, yD, '*','LineWidth', 1);
axis equal
legend('link 1', 'link 2', 'Wiper','Ground 1', 'Ground 2')
title('Velocity')
xlabel('vx[m/s]')
ylabel('vy[m/s]')