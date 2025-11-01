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
% Assume body extends length L_body from pivot (wheel axle)
% For a body with CoM at distance l, assume total length is 2*l
L_body = 2*l;  % Total body length extending from pivot

% Body tip height: h = r + L_body*cos(theta)
% Ground contact when: h = 0 => cos(theta) = -r/L_body
theta_limit = acos(-r/L_body);  % Limit angle in radians

fprintf('Ground contact limit angle: %.2f deg (%.4f rad)\n', ...
        rad2deg(theta_limit), theta_limit);

%% Pack parameters for ODE function
params = struct('d', d, 'l', l, 'r', r, 'mb', mb, 'mw', mw, ...
                'J', J, 'K', K, 'I1', I1, 'I2', I2, 'I3', I3, ...
                'calpha', calpha, 'g', g, 'theta_limit', theta_limit);

%% Initial conditions
% State vector: [x, theta, psi, dx, dtheta, dpsi]
% x: forward position, theta: pitch angle, psi: yaw angle
x0 = 0;          % Initial forward position (m)
theta0 = 0.1;    % Initial pitch angle (rad) - small perturbation from vertical
psi0 = 0;        % Initial yaw angle (rad)
dx0 = 0;         % Initial forward velocity (m/s)
dtheta0 = 0;     % Initial pitch angular velocity (rad/s)
dpsi0 = 0;       % Initial yaw angular velocity (rad/s)

initial_state = [x0; theta0; psi0; dx0; dtheta0; dpsi0];

%% Simulation parameters
t_final = 100;    % Simulation time (seconds)
tspan = [0 t_final];

%% Define control input (torques on left and right wheels)
% For now, use zero control to see open-loop behavior
control_func = @(t, state) [0; 0];  % [tau_L; tau_R]

%% Solve ODE with ground contact event detection
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, ...
                 'Events', @(t, y) ground_contact_event(t, y, theta_limit));
[t, state, te, ye, ie] = ode45(@(t, y) segway_dynamics(t, y, params, control_func), tspan, initial_state, options);

% Check if ground contact occurred
if ~isempty(te)
    fprintf('Ground contact detected at t = %.3f s, theta = %.2f deg\n', ...
            te(1), rad2deg(ye(1,2)));
end

%% Extract states
x = state(:, 1);
theta = state(:, 2);
psi = state(:, 3);
dx = state(:, 4);
dtheta = state(:, 5);
dpsi = state(:, 6);

%% Compute Z positions (vertical)
% Z position of center of mass
z_cm = r + l*cos(theta);

% Z position of body tip (top of pendulum)
z_tip = r + L_body*cos(theta);

%% Compute global X-Y position
% Transform local velocity to global frame using yaw angle
vx_global = dx .* cos(psi);  % velocity in global X direction
vy_global = dx .* sin(psi);  % velocity in global Y direction

% Integrate to get global position (starting from origin)
X_global = cumtrapz(t, vx_global);
Y_global = cumtrapz(t, vy_global);

%% Plot results
figure('Position', [100, 100, 1200, 800]);

subplot(3, 2, 1);
plot(t, x);
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('x [m]', 'interpreter', 'latex');
title('Position', 'interpreter', 'latex', 'FontSize', 14);
grid on;

subplot(3, 2, 2);
plot(t, dx);
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('$\dot{x}$ $\left[\frac{m}{s}\right]$', 'interpreter', 'latex');
title('Linear Velocity', 'interpreter', 'latex', 'FontSize', 14);
grid on;

subplot(3, 2, 3);
plot(t, theta * 180/pi);
hold on;
% Plot ground contact limits
yline(rad2deg(theta_limit), 'r--', 'LineWidth', 1.5);
yline(-rad2deg(theta_limit), 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('$\theta$ [deg]', 'interpreter', 'latex');
title('Pitch Angle (Tilt)', 'interpreter', 'latex', 'FontSize', 14);
grid on;
legend('Pitch Angle', 'Ground Limit', 'Location', 'best');

subplot(3, 2, 4);
plot(t, dtheta * 180/pi);
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('$\dot{\theta}$ $\left[\frac{deg}{s}\right]$', 'interpreter', 'latex');
title('Pitch Angular Velocity', 'interpreter', 'latex', 'FontSize', 14);
grid on;

subplot(3, 2, 5);
plot(t, psi * 180/pi);
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('$\psi$ [deg]', 'interpreter', 'latex');
title('Yaw Angle', 'interpreter', 'latex', 'FontSize', 14);
grid on;

subplot(3, 2, 6);
plot(t, dpsi * 180/pi);
xlabel('Time [s]', 'interpreter', 'latex');
ylabel('$\dot{\psi}$ $\left[\frac{deg}{s}\right]$', 'interpreter', 'latex');
title('Yaw Angular Velocity', 'interpreter', 'latex', 'FontSize', 14);
grid on;

sgtitle('Two-wheeled Inverted Pendulum Balancing Mobile Robot', 'interpreter', 'latex')

%% Plot Z positions and trajectories
% figure('Position', [150, 150, 1400, 500]);
% 
% % Z position vs time
% subplot(1, 3, 1);
% plot(t, z_cm, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Center of Mass');
% hold on;
% plot(t, z_tip, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Body Tip');
% yline(0, 'k--', 'Ground', 'LineWidth', 1.5);
% yline(r, 'g--', 'Wheel Axle', 'LineWidth', 1);
% hold off;
% xlabel('Time [s]', 'interpreter', 'latex');
% ylabel('Z Position [m]', 'interpreter', 'latex');
% title('Vertical Position (Height)', 'interpreter', 'latex', 'FontSize', 14);
% legend('Location', 'best');
% grid on;
% 
% % Side view trajectory (X-Z plane)
% subplot(1, 3, 2);
% plot(x, z_cm, 'b-', 'LineWidth', 1.5, 'DisplayName', 'CoM Trajectory');
% hold on;
% plot(x, z_tip, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Tip Trajectory');
% plot(x(1), z_cm(1), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
% plot(x(end), z_cm(end), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'End');
% yline(0, 'k--', 'Ground', 'LineWidth', 1.5);
% hold off;
% xlabel('X (Local) [m]', 'interpreter', 'latex');
% ylabel('Z Position [m]', 'interpreter', 'latex');
% title('Side View (Local Frame)', 'interpreter', 'latex', 'FontSize', 14);
% legend('Location', 'best');
% grid on;
% axis equal;
% 
% % Top view trajectory (X-Y plane in global frame)
% subplot(1, 3, 3);
% plot(X_global, Y_global, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');
% hold on;
% plot(X_global(1), Y_global(1), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
% plot(X_global(end), Y_global(end), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'End');
% hold off;
% xlabel('X (Global) [m]', 'interpreter', 'latex');
% ylabel('Y (Global) [m]', 'interpreter', 'latex');
% title('Top View (Global Frame)', 'interpreter', 'latex', 'FontSize', 14);
% legend('Location', 'best');
% grid on;
% axis equal;