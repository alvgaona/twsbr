clear; clc; close all;

%% Parameters of the model
d = 0.59;        % Distance between the two wheels
l = 0.14;        % Length of the pendulum
r = 0.2;         % Radius of wheels
mb = 41;         % Mass of the pendulum body (except wheels)
mw = 2;          % Mass of each wheel
J = 0.04;        % Moment of inertia w.r.t wheel axis
K = 0.02;        % Moment of inertia w.r.t vertical axis
I1 = 1;          % Moment of inertia of the pendulum body
I2 = 1;          % Moment of inertia of the pendulum body
I3 = 1;          % Moment of inertia of the pendulum body
calpha = 0.01;   % Coefficient of viscous friction on the wheel axis
g = 9.81;        % Gravitational acceleration (m/s^2)

%% Calculate ground contact limit angle
L_body = 2*l;  % Total body length extending from pivot
theta_limit = acos(-r/L_body);  % Limit angle in radians

fprintf('Ground contact limit angle: %.2f deg (%.4f rad)\n', ...
        rad2deg(theta_limit), theta_limit);

%% Linearization around theta = 0 (upright position)
fprintf('\n--- Linearizing dynamics around theta = 0 ---\n');

% Mass matrix at theta = 0 (constant)
a11 = mb + 2*mw + 2*J/r^2;
a12 = mb*l;  % cos(0) = 1
a21 = a12;
a22 = I2 + mb*l^2;
a33 = I3 + 2*K + mw*d^2/2 + J*d^2/(2*r^2);  % sin(0)^2 = 0

M0 = [
    a11 a12 0;
    a21 a22 0;
    0 0 a33
];

% Damping matrix (already constant)
d11 = 2*calpha/r^2;
d12 = -2*calpha/r;
d21 = d12;
d22 = 2*calpha;
d33 = calpha*d^2/(2*r^2);

D = [
    d11 d12 0;
    d21 d22 0;
    0 0 d33
];

% Input matrix (constant)
b11 = 1/r;
b12 = b11;
b21 = -1;
b22 = b21;
b31 = -d / (2*r);
b32 = -b31;

B_input = [
    b11 b12;
    b21 b22;
    b31 b32;
];

% Gravity linearization: G = [0; -mb*l*g*theta; 0]
% G_lin * q gives the gravity vector
G_lin = [
    0    0         0;
    0   -mb*l*g    0;
    0    0         0;
];

%% State-space representation
% State vector: x_state = [x; theta; psi; dx; dtheta; dpsi]
% Dynamics: qddot = M0^(-1) * (B*u - D*qdot - G_lin*q)
%
% dx_state/dt = [qdot; qddot] = [qdot; M0^(-1)*(B*u - D*qdot - G_lin*q)]
%
% A * x_state + B_ss * u

M0_inv = inv(M0);

% State matrix A (6x6)
A = [zeros(3,3),        eye(3);
     -M0_inv*G_lin,    -M0_inv*D];

% Input matrix B (6x2)
B_ss = [zeros(3,2);
        M0_inv*B_input];

fprintf('\nState matrix A (6x6):\n');
disp(A);

fprintf('Input matrix B_ss (6x2):\n');
disp(B_ss);

%% Check controllability
C_matrix = ctrb(A, B_ss);
rank_C = rank(C_matrix);
fprintf('\nControllability matrix rank: %d (should be 6 for full controllability)\n', rank_C);

if rank_C < 6
    warning('System is not fully controllable!');
else
    fprintf('System is fully controllable.\n');
end

%% Check open-loop stability
eigenvalues_open = eig(A);
fprintf('\nOpen-loop eigenvalues:\n');
disp(eigenvalues_open);

unstable_modes = sum(real(eigenvalues_open) > 0);
fprintf('Number of unstable modes: %d\n', unstable_modes);

%% Design LQR controller
% Cost function: J = integral(x'Qx + u'Ru) dt
% We want to:
% - Keep theta (pitch) near 0 - high weight
% - Keep x position from drifting too much - moderate weight
% - Keep psi (yaw) from drifting - low weight (or zero for now)
% - Penalize velocities moderately
% - Keep control effort reasonable

% State weights Q (6x6)
% Order: [x, theta, psi, dx, dtheta, dpsi]
Q = diag([
    1,      % x position - moderate
    100,    % theta - HIGH (most important!)
    0.1,    % psi - low
    1,      % dx
    10,     % dtheta - penalize pitch rate
    0.1     % dpsi
]);

% Control effort weights R (2x2)
% [tau_L, tau_R]
R = diag([0.1, 0.1]);  % Relatively small - allow large control effort

fprintf('\n--- Designing LQR Controller ---\n');
fprintf('State weights Q (diagonal):\n');
disp(diag(Q)');
fprintf('Control weights R (diagonal):\n');
disp(diag(R)');

% Compute LQR gain
[K_lqr, S, poles_closed] = lqr(A, B_ss, Q, R);

fprintf('\nLQR gain matrix K (2x6):\n');
disp(K_lqr);

fprintf('\nClosed-loop poles:\n');
disp(poles_closed);

fprintf('All closed-loop poles have negative real parts: %d\n', all(real(poles_closed) < 0));

%% Pack parameters for simulation
params = struct('d', d, 'l', l, 'r', r, 'mb', mb, 'mw', mw, ...
                'J', J, 'K', K, 'I1', I1, 'I2', I2, 'I3', I3, ...
                'calpha', calpha, 'g', g, 'theta_limit', theta_limit, ...
                'K_lqr', K_lqr);

%% Initial conditions
x0 = 0;          % Initial forward position (m)
theta0 = 0.5;    % Initial pitch angle (rad) - small perturbation from vertical
psi0 = 0;        % Initial yaw angle (rad)
dx0 = 0;         % Initial forward velocity (m/s)
dtheta0 = 0;     % Initial pitch angular velocity (rad/s)
dpsi0 = 0;       % Initial yaw angular velocity (rad/s)

initial_state = [x0; theta0; psi0; dx0; dtheta0; dpsi0];

fprintf('\n--- Initial Conditions ---\n');
fprintf('Initial pitch angle: %.2f deg\n', rad2deg(theta0));

%% Simulation parameters
t_final = 100;    % Simulation time (seconds)
tspan = [0 t_final];

%% Define LQR control law
% u = -K * x (state feedback)
control_func_lqr = @(t, state) -params.K_lqr * state;

%% Solve ODE with LQR control
fprintf('\n--- Running Closed-Loop Simulation with LQR ---\n');
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, ...
                 'Events', @(t, y) ground_contact_event(t, y, theta_limit));
[t, state, te, ye, ie] = ode45(@(t, y) segway_dynamics(t, y, params, control_func_lqr), ...
                                tspan, initial_state, options);

% Check if ground contact occurred
if ~isempty(te)
    fprintf('Ground contact detected at t = %.3f s, theta = %.2f deg\n', ...
            te(1), rad2deg(ye(1,2)));
else
    fprintf('No ground contact - robot stabilized successfully!\n');
end

%% Extract states
x = state(:, 1);
theta = state(:, 2);
psi = state(:, 3);
dx = state(:, 4);
dtheta = state(:, 5);
dpsi = state(:, 6);

% Compute control inputs over time
u = zeros(length(t), 2);
for i = 1:length(t)
    u(i,:) = control_func_lqr(t(i), state(i,:)');
end

%% Compute Z positions (vertical)
z_cm = r + l*cos(theta);
z_tip = r + L_body*cos(theta);

%% Compute global X-Y position
vx_global = dx .* cos(psi);
vy_global = dx .* sin(psi);
X_global = cumtrapz(t, vx_global);
Y_global = cumtrapz(t, vy_global);

%% Plot results
figure('Position', [100, 100, 1400, 900], 'Color', 'w');
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');

subplot(3, 3, 1);
plot(t, x, 'LineWidth', 2);
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('$x$ [m]', 'FontSize', 14);
title('Position', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);

subplot(3, 3, 2);
plot(t, rad2deg(theta), 'LineWidth', 2);
hold on;
yline(rad2deg(theta_limit), 'r--', 'LineWidth', 1.5);
yline(-rad2deg(theta_limit), 'r--', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1);
hold off;
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('$\theta$ [$^\circ$]', 'FontSize', 14);
title('Pitch Angle', 'FontSize', 14);
grid on;
legend('$\theta(t)$', 'Ground Limit', '', 'Target', 'Location', 'best');
set(gca, 'FontSize', 12);

subplot(3, 3, 3);
plot(t, psi * 180/pi);
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('$\psi$ [deg]', 'interpreter', 'latex');
title('Yaw Angle', 'interpreter', 'latex', 'FontSize', 12);
grid on;

subplot(3, 3, 4);
plot(t, dx);
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('$\dot{x}$ [m/s]', 'interpreter', 'latex');
title('Linear Velocity', 'interpreter', 'latex', 'FontSize', 12);
grid on;

subplot(3, 3, 5);
plot(t, dtheta * 180/pi);
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('$\dot{\theta}$ [deg/s]', 'interpreter', 'latex');
title('Pitch Angular Velocity', 'interpreter', 'latex', 'FontSize', 12);
grid on;

subplot(3, 3, 6);
plot(t, dpsi * 180/pi);
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('$\dot{\psi}$ [deg/s]', 'interpreter', 'latex');
title('Yaw Angular Velocity', 'interpreter', 'latex', 'FontSize', 12);
grid on;

subplot(3, 3, 7);
plot(t, u(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', '\tau_L (Left)');
hold on;
plot(t, u(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', '\tau_R (Right)');
hold off;
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('Torque [Nm]', 'interpreter', 'latex');
title('Control Inputs', 'interpreter', 'latex', 'FontSize', 12);
legend('Location', 'best');
grid on;

subplot(3, 3, 8);
plot(t, z_cm, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Center of Mass');
hold on;
plot(t, z_tip, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Body Tip');
yline(0, 'k--', 'Ground', 'LineWidth', 1.5);
yline(r, 'g--', 'Wheel Axle', 'LineWidth', 1);
hold off;
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('Z Position [m]', 'interpreter', 'latex');
title('Vertical Position', 'interpreter', 'latex', 'FontSize', 12);
legend('Location', 'best');
grid on;

subplot(3, 3, 9);
plot(X_global, Y_global, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');
hold on;
plot(X_global(1), Y_global(1), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
plot(X_global(end), Y_global(end), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'End');
hold off;
xlabel('X (Global) [m]', 'interpreter', 'latex');
ylabel('Y (Global) [m]', 'interpreter', 'latex');
title('Top View (Global Frame)', 'interpreter', 'latex', 'FontSize', 12);
legend('Location', 'best');
grid on;
axis equal;

sgtitle('LQR-Controlled Two-Wheeled Inverted Pendulum', 'FontSize', 18, 'FontWeight', 'bold');

%% Performance metrics
fprintf('\n--- Performance Metrics ---\n');
fprintf('Final pitch angle: %.4f deg\n', rad2deg(theta(end)));
fprintf('Final position: %.4f m\n', x(end));
fprintf('Max control torque: %.2f Nm\n', max(abs(u(:))));
fprintf('Settling time (|theta| < 1 deg): ');

settling_threshold = deg2rad(1);  % 1 degree
settling_idx = find(abs(theta) < settling_threshold, 1, 'first');
if ~isempty(settling_idx)
    fprintf('%.3f s\n', t(settling_idx));
else
    fprintf('Not achieved\n');
end
