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
L_body = 2*l;                     % Total body length extending from pivot
theta_limit = acos(-r/L_body);    % Limit angle in radians

fprintf('Ground contact limit angle: %.2f deg (%.4f rad)\n', ...
        rad2deg(theta_limit), theta_limit);

%% Pack parameters
params = struct('d', d, 'l', l, 'r', r, 'mb', mb, 'mw', mw, ...
                'J', J, 'K', K, 'I1', I1, 'I2', I2, 'I3', I3, ...
                'calpha', calpha, 'g', g, 'theta_limit', theta_limit);

%% NMPC Controller Setup

% Create NMPC object
nx = 6;  % Number of states: [x, θ, ψ, dx, dθ, dψ]
ny = 6;  % Number of outputs (measuring all states)
nu = 2;  % Number of inputs: [τL, τR]

nlobj = nlmpc(nx, ny, nu);

% Sampling time and horizons
Ts = 0.05;                      % Sampling time (50 ms)
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 20;   % Prediction horizon (1 second)
nlobj.ControlHorizon = 5;       % Control horizon

fprintf('Sampling time: %.3f s\n', Ts);
fprintf('Prediction horizon: %d steps (%.2f s)\n', ...
    nlobj.PredictionHorizon, nlobj.PredictionHorizon*Ts);
fprintf('Control horizon: %d steps (%.2f s)\n', ...
    nlobj.ControlHorizon, nlobj.ControlHorizon*Ts);

%% Define state and input names
nlobj.States(1).Name = 'x';
nlobj.States(2).Name = 'theta';
nlobj.States(3).Name = 'psi';
nlobj.States(4).Name = 'dx';
nlobj.States(5).Name = 'dtheta';
nlobj.States(6).Name = 'dpsi';

nlobj.ManipulatedVariables(1).Name = 'tau_L';
nlobj.ManipulatedVariables(2).Name = 'tau_R';

%% Input constraints
max_torque = 10; % Maximum torque per wheel (Nm)
nlobj.ManipulatedVariables(1).Min = -max_torque;
nlobj.ManipulatedVariables(1).Max = max_torque;
nlobj.ManipulatedVariables(2).Min = -max_torque;
nlobj.ManipulatedVariables(2).Max = max_torque;

max_torque_rate = 100; % Μaximum torque rate per wheel (Nm/s)
nlobj.ManipulatedVariables(1).RateMin = -max_torque_rate * Ts;
nlobj.ManipulatedVariables(1).RateMax = max_torque_rate * Ts;
nlobj.ManipulatedVariables(2).RateMin = -max_torque_rate * Ts;
nlobj.ManipulatedVariables(2).RateMax = max_torque_rate * Ts;

fprintf('\nInput constraints:\n');
fprintf('  Max torque: ±%.1f Nm\n', max_torque);
fprintf('  Max torque rate: ±%.1f Nm/s\n', max_torque_rate);

%% State constraints
max_theta = deg2rad(90);

% Keep robot from falling
nlobj.States(2).Min = -max_theta;
nlobj.States(2).Max = max_theta;

fprintf('State constraints:\n');
fprintf('  Pitch angle: ±%.1f deg\n', rad2deg(max_theta));

mpc_params = struct('d', d+d*0.1, 'l', l+0.1*l, 'r', r-0.1*r, 'mb', mb + 0.1*mb, 'mw', mw-0.1*mw, ...
                'J', J+0.1*J, 'K', K-0.1*K, 'I1', I1+0.1*I1, 'I2', I2-0.1*I2, 'I3', I3+0.1*I3, ...
                'calpha', calpha+0.1*calpha, 'g', g+0.1*g, 'theta_limit', theta_limit);

%% Measurement noise setup
fprintf('\n--- Measurement Noise Setup ---\n');

% Define noise standard deviations for each state
noise_std = [
    0.01,   % x position noise (m)
    0.005,  % θ pitch noise (rad) ~0.3°
    0.005,  % ψ yaw noise (rad) ~0.3°
    0.02,   % dx velocity noise (m/s)
    0.01,   % dθ pitch rate noise (rad/s)
    0.01    % dψ yaw rate noise (rad/s)
];

fprintf('Measurement noise (1-sigma):\n');
fprintf('  Position: %.3f m\n', noise_std(1));
fprintf('  Pitch angle: %.3f rad (%.2f deg)\n', noise_std(2), rad2deg(noise_std(2)));
fprintf('  Yaw angle: %.3f rad (%.2f deg)\n', noise_std(3), rad2deg(noise_std(3)));
fprintf('  Linear velocity: %.3f m/s\n', noise_std(4));
fprintf('  Pitch rate: %.3f rad/s (%.2f deg/s)\n', noise_std(5), rad2deg(noise_std(5)));
fprintf('  Yaw rate: %.3f rad/s (%.2f deg/s)\n', noise_std(6), rad2deg(noise_std(6)));

%% Define prediction model (nonlinear dynamics)
nlobj.Model.StateFcn = @(x, u) twsbr_discrete_dynamics(x, u, Ts, mpc_params);
nlobj.Model.IsContinuousTime = false;

% Output function (full state measurement)
nlobj.Model.OutputFcn = @(x, u) x;  % y = x

%% Define cost function (stage cost)
Q_nmpc = diag([
    100,      ... x [no position tracking]
    100,    ... θ [high - stay upright]
    100,    ... ψ [high - lock heading at desired yaw]
    0,      ... dx [no velocity tracking]
    10,     ... dθ [penalize pitch rate]
    20      ... dψ [penalize yaw drift]
]);

R_nmpc = diag([0.1, 0.1]);

% Set weights
nlobj.Weights.OutputVariables = diag(Q_nmpc)'; 
nlobj.Weights.ManipulatedVariables = diag(R_nmpc)';
nlobj.Weights.ManipulatedVariablesRate = 0.1*diag(R_nmpc)';

fprintf('\nCost function weights:\n');
fprintf('  State weights Q (diagonal): ');
fprintf('%.1f ', diag(Q_nmpc));
fprintf('\n');
fprintf('  Control weights R (diagonal): ');
fprintf('%.1f ', diag(R_nmpc));
fprintf('\n');

%% Validate NMPC object
fprintf('\nValidating NMPC controller...\n');
x0_test = [0; 0.1; 0; 0; 0; 0];
u0_test = [0; 0];
validateFcns(nlobj, x0_test, u0_test);
fprintf('NMPC controller validated successfully!\n');

%% Initial conditions
x0 = 0;          % Initial forward position (m) - start at origin
theta0 = 0.4;    % Initial pitch angle (rad) - small perturbation (~5.7 deg)
psi0 = 3.14;        % Initial yaw angle (rad)
dx0 = 0;         % Initial forward velocity (m/s)
dtheta0 = 0;     % Initial pitch angular velocity (rad/s)
dpsi0 = 0;       % Initial yaw angular velocity (rad/s)

x_init = [x0; theta0; psi0; dx0; dtheta0; dpsi0];

fprintf('Initial position: %.2f m\n', x0);
fprintf('Initial pitch angle: %.2f deg\n', rad2deg(theta0));

%% Simulation parameters
t_final = 20;    % Simulation time (seconds)
N_steps = round(t_final / Ts);

%% Drift model setup (random walk)

drift_std = [
    0.005,   % x position drift rate (m/√s)
    0.003,   % θ pitch drift rate (rad/√s)
    0.003,   % ψ yaw drift rate (rad/√s)
    0.01,    % dx velocity drift rate (m/s/√s)
    0.005,   % dθ pitch rate drift rate (rad/s/√s)
    0.005    % dψ yaw rate drift rate (rad/s/√s)
];

fprintf('Drift rates (random walk):\n');
fprintf('  Position: %.4f m/√s\n', drift_std(1));
fprintf('  Pitch angle: %.4f rad/√s (%.3f deg/√s)\n', drift_std(2), rad2deg(drift_std(2)));
fprintf('  Yaw angle: %.4f rad/√s (%.3f deg/√s)\n', drift_std(3), rad2deg(drift_std(3)));
fprintf('  Linear velocity: %.4f m/s/√s\n', drift_std(4));
fprintf('  Pitch rate: %.4f rad/s/√s (%.3f deg/s/√s)\n', drift_std(5), rad2deg(drift_std(5)));
fprintf('  Yaw rate: %.4f rad/s/√s (%.3f deg/s/√s)\n', drift_std(6), rad2deg(drift_std(6)));

fprintf('\nExpected drift magnitude after %.0f seconds (3σ):\n', t_final);
fprintf('  Position: ±%.4f m\n', 3*drift_std(1)*sqrt(t_final));
fprintf('  Pitch angle: ±%.4f rad (%.2f deg)\n', 3*drift_std(2)*sqrt(t_final), rad2deg(3*drift_std(2)*sqrt(t_final)));
fprintf('  Yaw angle: ±%.4f rad (%.2f deg)\n', 3*drift_std(3)*sqrt(t_final), rad2deg(3*drift_std(3)*sqrt(t_final)));

% Preallocate arrays
t_sim = (0:N_steps) * Ts;
x_hist = zeros(nx, N_steps+1);
x_measured_hist = zeros(nx, N_steps+1);  % Store noisy measurements
u_hist = zeros(nu, N_steps);

% Initialize drift (random walk) - starts at zero
drift = zeros(nx, 1);
drift_hist = zeros(nx, N_steps+1);

x_hist(:, 1) = x_init;
x_measured_hist(:, 1) = x_init;  % Initial measurement (no noise)
drift_hist(:, 1) = drift;

%% Trajectory selection
trajectory_type = 'ramp';  % 'sinusoid', 'step', 'ramp', 'figure8'

switch trajectory_type
    case 'sinusoid'
        A_x = 3.0;
        omega = 0.3;
        x_ref_func = @(t) [A_x*sin(omega*t), 0, 0, A_x*omega*cos(omega*t), 0, 0];
        fprintf('\n--- Reference Trajectory: Sinusoid ---\n');
        fprintf('x(t) = %.1f*sin(%.2f*t)\n', A_x, omega);
        fprintf('Period: %.2f seconds\n', 2*pi/omega);
        fprintf('Max velocity: %.2f m/s\n', A_x*omega);
        fprintf('Max acceleration: %.2f m/s^2\n', A_x*omega^2);
    case 'step'
        x_ref_func = @(t) [(t >= 2)*2.0 + (t >= 5)*(-2.0), 0, 0, 0, 0, 0];
        fprintf('\n--- Reference Trajectory: Steps ---\n');
        fprintf('x(t): 0 → 2 @ t=2s, 2 → 0 @ t=5s\n');
    case 'ramp'
        v_ref = 0.5;    % Reference velocity (m/s)
        x_ref_func = @(t) [v_ref*t, 0, 0, v_ref, 0, 0];
        fprintf('\n--- Reference Trajectory: Ramp ---\n');
        fprintf('x(t) = %.2f*t (constant velocity)\n', v_ref);
    case 'figure8'
        A_x = 2.0;
        A_y = 1.0;
        omega = 0.3;
        x_ref_func = @(t) [A_x*sin(omega*t), 0, 0, A_x*omega*cos(omega*t), 0, 0];
        fprintf('\n--- Reference Trajectory: Figure-8 ---\n');
        fprintf('Complex trajectory in XY plane\n');
end

%% Run NMPC simulation
fprintf('\n--- Running Closed-Loop Simulation with NMPC ---\n');
fprintf('Simulating %.1f seconds (%d steps)...\n', t_final, N_steps);

% Initial control
u_prev = [0; 0];

% Store reference trajectory for plotting
x_ref_hist = zeros(ny, N_steps+1);

% Create waitbar
h_wait = waitbar(0, 'Running NMPC simulation...');
set(findall(h_wait, 'Type', 'text'), 'Interpreter', 'none');

for k = 1:N_steps
    % Update waitbar
    if mod(k, 10) == 0
        waitbar(k/N_steps, ...
            h_wait, ...
            sprintf('Running NMPC simulation... %.1f%%', ...
            100*k/N_steps));
    end

    % Current ground truth (from simulation)
    x_true = x_hist(:, k);

    % Update drift (random walk)
    drift_increment = sqrt(Ts) * drift_std .* randn(nx, 1);
    drift = drift + drift_increment;
    drift_hist(:, k) = drift;

    % Add white noise
    white_noise = noise_std .* randn(nx, 1);

    % Total measurement error = white noise + drift
    x_measured = x_true + white_noise + drift;

    % Store measured state
    x_measured_hist(:, k) = x_measured;

    % Get current reference from trajectory
    x_ref = x_ref_func(t_sim(k));
    x_ref_hist(:, k) = x_ref';

    % Compute optimal control with noisy measurements
    [u_opt, ~] = nlmpcmove(nlobj, x_measured, u_prev, x_ref, []);

    % Store control input
    u_hist(:, k) = u_opt;

    % Apply control with true state
    x_next = twsbr_discrete_dynamics(x_true, u_opt, Ts, params);

    % Check for ground contact
    if abs(x_next(2)) > theta_limit
        fprintf('\nGround contact detected at t = %.3f s, theta = %.2f deg\n', ...
                t_sim(k+1), rad2deg(x_next(2)));
        % Truncate arrays
        x_hist = x_hist(:, 1:k+1);
        x_measured_hist = x_measured_hist(:, 1:k+1);
        drift_hist = drift_hist(:, 1:k+1);
        u_hist = u_hist(:, 1:k);
        x_ref_hist = x_ref_hist(:, 1:k+1);
        t_sim = t_sim(1:k+1);
        break;
    end

    % Store next ground truth state
    x_hist(:, k+1) = x_next;

    % Update previous control
    u_prev = u_opt;
end

close(h_wait);
fprintf('Simulation completed.\n');

% Store final reference
x_ref_hist(:, end) = x_ref_func(t_sim(end))';

% Update final drift
drift_increment = sqrt(Ts) * drift_std .* randn(nx, 1);
drift = drift + drift_increment;
drift_hist(:, end) = drift;

% Store final measurement with white noise + drift
white_noise = noise_std .* randn(nx, 1);
x_measured_hist(:, end) = x_hist(:, end) + white_noise + drift;

%% Extract states
x = x_hist(1, :)';
theta = x_hist(2, :)';
psi = x_hist(3, :)';
dx = x_hist(4, :)';
dtheta = x_hist(5, :)';
dpsi = x_hist(6, :)';
t = t_sim';

% Extract reference trajectory
x_ref_traj = x_ref_hist(1, :)';
dx_ref_traj = x_ref_hist(4, :)';

% Wrap yaw angle for plotting
psi_wrapped = mod(psi + pi, 2*pi) - pi;

%% Compute global X-Y position (use wrapped yaw)
vx_global = dx .* cos(psi_wrapped);
vy_global = dx .* sin(psi_wrapped);
X_global = cumtrapz(t, vx_global);
Y_global = cumtrapz(t, vy_global);

%% Plot results
figure('Position', [100, 100, 1400, 900], 'Color', 'w');
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');

subplot(3, 3, 1);
plot(t, x, 'LineWidth', 2, 'DisplayName', 'Position');
hold on;
plot(t, x_ref_traj, 'LineWidth', 2, 'DisplayName', 'Ref Position');
hold off;
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('x [m]', 'FontSize', 14);
title('Position (x)', 'FontSize', 14);
grid on;
legend('Location', 'best');
set(gca, 'FontSize', 12);

subplot(3, 3, 2);
plot(t, rad2deg(theta), 'LineWidth', 2, 'DisplayName', 'Pitch');
hold on;
yline(rad2deg(theta_limit), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ground Contact');
yline(-rad2deg(theta_limit), 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(rad2deg(max_theta), 'm--', 'LineWidth', 1, 'DisplayName', 'State Constraint');
yline(-rad2deg(max_theta), 'm--', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(0, 'k--', 'LineWidth', 1, 'DisplayName', 'Target');
hold off;
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('$\theta$ [$^\circ$]', 'FontSize', 14);
title('Pitch Angle (Tilt)', 'FontSize', 14);
grid on;
legend('Location', 'best');
set(gca, 'FontSize', 12);

subplot(3, 3, 3);
plot(t, rad2deg(psi_wrapped), 'LineWidth', 2);
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('$\psi$ [$^\circ$]', 'FontSize', 14);
title('Yaw Angle (wrapped)', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);

subplot(3, 3, 4);
plot(t, dx, 'b-', 'LineWidth', 2);
hold on;
plot(t, dx_ref_traj, 'r--', 'LineWidth', 2);
hold off;
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('$\dot{x}$ [m/s]', 'FontSize', 14);
title('Linear Velocity', 'FontSize', 14);
legend({'$\dot{x}$ (actual)', '$\dot{x}_{\mathrm{ref}}$'}, 'Location', 'best');
grid on;
set(gca, 'FontSize', 12);

subplot(3, 3, 5);
plot(t, rad2deg(dtheta), 'LineWidth', 2);
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('$\dot{\theta}$ [$^\circ$/s]', 'FontSize', 14);
title('Pitch Angular Velocity', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);

subplot(3, 3, 6);
plot(t, rad2deg(dpsi), 'LineWidth', 2);
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('$\dot{\psi}$ [$^\circ$/s]', 'FontSize', 14);
title('Yaw Angular Velocity', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);

subplot(3, 3, 7);
stairs(t(1:end-1), u_hist(1,:), 'b-', 'LineWidth', 2, 'DisplayName', '$\tau_L$ (Left)');
hold on;
stairs(t(1:end-1), u_hist(2,:), 'r-', 'LineWidth', 2, 'DisplayName', '$\tau_R$ (Right)');
yline(max_torque, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(-max_torque, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
hold off;
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('Torque [Nm]', 'FontSize', 14);
title('Control Inputs', 'FontSize', 14);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12);

sgtitle('NMPC-Controlled Two-Wheeled Self-Balancing Robot', 'FontSize', 18, 'FontWeight', 'bold');

%% Extract measured states
x_measured = x_measured_hist(1, :)';
theta_measured = x_measured_hist(2, :)';
psi_measured = x_measured_hist(3, :)';
dx_measured = x_measured_hist(4, :)';
dtheta_measured = x_measured_hist(5, :)';
dpsi_measured = x_measured_hist(6, :)';

%% Extract drift components
drift_x = drift_hist(1, :)';
drift_theta = drift_hist(2, :)';

%% Compare white noise vs drift contributions
figure('Position', [600, 400, 1400, 600], 'Color', 'w');

% Compute total measurement error components
total_error = x_measured_hist - x_hist;
white_noise_component_x = total_error(1, :)' - drift_x;
white_noise_component_theta = total_error(2, :)' - drift_theta;

subplot(2, 2, 1);
plot(t, total_error(1, :), 'k-', 'LineWidth', 2, 'DisplayName', 'Total Error');
hold on;
plot(t, drift_x, 'r-', 'LineWidth', 2, 'DisplayName', 'Drift');
plot(t, white_noise_component_x, 'b.', 'MarkerSize', 3, 'DisplayName', 'White Noise');
yline(0, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
hold off;
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('Error [m]', 'FontSize', 14);
title('Position Error: White Noise + Drift', 'FontSize', 14);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12);

subplot(2, 2, 2);
plot(t, rad2deg(total_error(2, :)), 'k-', 'LineWidth', 2, 'DisplayName', 'Total Error');
hold on;
plot(t, rad2deg(drift_theta), 'r-', 'LineWidth', 2, 'DisplayName', 'Drift');
plot(t, rad2deg(white_noise_component_theta), 'b.', 'MarkerSize', 3, 'DisplayName', 'White Noise');
yline(0, 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
hold off;
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('Error [$^\circ$]', 'FontSize', 14);
title('Pitch Angle Error: White Noise + Drift', 'FontSize', 14);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12);

subplot(2, 2, 3);
% Plot standard deviation over time
window_size = 20;  % Moving window for std calculation
std_drift_x = movstd(drift_x, window_size);
std_white_x = movstd(white_noise_component_x, window_size);

plot(t, std_drift_x, 'r-', 'LineWidth', 2, 'DisplayName', 'Drift Std (moving)');
hold on;
plot(t, std_white_x, 'b-', 'LineWidth', 2, 'DisplayName', 'White Noise Std (moving)');
plot(t, noise_std(1)*ones(size(t)), 'b--', 'LineWidth', 1.5, 'DisplayName', 'White Noise $\sigma$ (theory)');
plot(t, drift_std(1)*sqrt(t), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Drift $\sigma(t)$ (theory)');
hold off;
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('Standard Deviation [m]', 'FontSize', 14);
title('Position Error Standard Deviation vs Time', 'FontSize', 14);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12);

subplot(2, 2, 4);
final_idx = length(t);
histogram(white_noise_component_x, 30, 'Normalization', 'pdf', 'DisplayName', 'White Noise', 'FaceAlpha', 0.5);
hold on;
x_range = linspace(-4*noise_std(1), 4*noise_std(1), 100);
plot(x_range, normpdf(x_range, 0, noise_std(1)), 'b-', 'LineWidth', 2, 'DisplayName', 'Theory ($\mathcal{N}(0, \sigma^2)$)');
hold off;
xlabel('Error [m]', 'FontSize', 14);
ylabel('Probability Density', 'FontSize', 14);
title('White Noise Distribution (Position)', 'FontSize', 14);
legend('Location', 'best');
grid on;
set(gca, 'FontSize', 12);

sgtitle('White Noise vs Drift Comparison', 'FontSize', 18, 'FontWeight', 'bold');

%% Helper function: Discrete-time dynamics
function x_next = twsbr_discrete_dynamics(x, u, Ts, params)
    % One-step Euler integration of continuous dynamics
    % More accurate methods (RK4, ode45) could be used
    
    % Continuous dynamics
    dx_dt = twsbr_dynamics_continuous(x, u, params);
    
    % Euler step
    x_next = x + Ts * dx_dt;
end

%% Helper function: Continuous dynamics
function dx_dt = twsbr_dynamics_continuous(x, u, params)
    % Extract states
    theta = x(2);
    dx = x(4);
    dtheta = x(5);
    dpsi = x(6);

    % Extract parameters
    d = params.d;
    l = params.l;
    r = params.r;
    mb = params.mb;
    mw = params.mw;
    J = params.J;
    K = params.K;
    I1 = params.I1;
    I2 = params.I2;
    I3 = params.I3;
    calpha = params.calpha;
    g = params.g;

    % Mass matrix
    a11 = mb + 2*mw + 2*J/r^2;
    a12 = mb*l*cos(theta);
    a21 = a12;
    a22 = I2 + mb*l^2;
    a33 = I3 + 2*K + mw*d^2/2 + J*d^2/(2*r^2) - (I3 - I1 - mb*l^2)*(sin(theta))^2;

    M = [
        a11 a12 0;
        a21 a22 0;
        0 0 a33
    ];

    % Coriolis matrix
    c12 = -mb*l*dtheta*sin(theta);
    c13 = -mb*l*dpsi*sin(theta);
    c23 = (I3 - I1 - mb*l^2)*dpsi*sin(theta)*cos(theta);
    c31 = mb*l*dpsi*sin(theta);
    c32 = -(I3-I1-mb*l^2)*dpsi*sin(theta)*cos(theta);
    c33 = -(I3-I1-mb*l^2)*dtheta*sin(theta)*cos(theta);

    C = [
        0 c12 c13;
        0 0 c23;
        c31 c32 c33
    ];

    % Damping matrix
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

    % Input matrix
    b11 = 1/r;
    b12 = b11;
    b21 = -1;
    b22 = b21;
    b31 = -d / (2*r);
    b32 = -b31;

    B = [
        b11 b12;
        b21 b22;
        b31 b32;
    ];

    % Gravity vector
    g2 = -mb*l*g*sin(theta);
    G = [0; g2; 0];

    % Velocity vector
    qdot = [dx; dtheta; dpsi];

    % Equation of motion: M*qddot + C*qdot + D*qdot + G = B*u
    qddot = M \ (B*u - C*qdot - D*qdot - G);

    % Construct derivative of state vector
    dx_dt = [
        qdot;
        qddot
    ];
end
