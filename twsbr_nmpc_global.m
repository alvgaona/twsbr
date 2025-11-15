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
L_body = 2*l;
theta_limit = acos(-r/L_body);

fprintf('Ground contact limit angle: %.2f deg (%.4f rad)\n', ...
        rad2deg(theta_limit), theta_limit);

%% Pack parameters
params = struct('d', d, 'l', l, 'r', r, 'mb', mb, 'mw', mw, ...
                'J', J, 'K', K, 'I1', I1, 'I2', I2, 'I3', I3, ...
                'calpha', calpha, 'g', g, 'theta_limit', theta_limit);

%% NMPC Controller Setup
fprintf('\n--- Setting up NMPC Controller ---\n');

nx = 6;  % States: [x, θ, ψ, dx, dθ, dψ]
ny = 6;  % Outputs (measuring all states)
nu = 2;  % Inputs: [τL, τR]

nlobj = nlmpc(nx, ny, nu);

Ts = 0.1;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 25;
nlobj.ControlHorizon = 10;

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
max_torque = 50;
nlobj.ManipulatedVariables(1).Min = -max_torque;
nlobj.ManipulatedVariables(1).Max = max_torque;
nlobj.ManipulatedVariables(2).Min = -max_torque;
nlobj.ManipulatedVariables(2).Max = max_torque;

max_torque_rate = 200;
nlobj.ManipulatedVariables(1).RateMin = -max_torque_rate * Ts;
nlobj.ManipulatedVariables(1).RateMax = max_torque_rate * Ts;
nlobj.ManipulatedVariables(2).RateMin = -max_torque_rate * Ts;
nlobj.ManipulatedVariables(2).RateMax = max_torque_rate * Ts;

fprintf('\nInput constraints:\n');
fprintf('  Max torque: ±%.1f Nm\n', max_torque);
fprintf('  Max torque rate: ±%.1f Nm/s\n', max_torque_rate);

%% State constraints
max_theta = deg2rad(30);
nlobj.States(2).Min = -max_theta;
nlobj.States(2).Max = max_theta;

fprintf('State constraints:\n');
fprintf('  Pitch angle: ±%.1f deg\n', rad2deg(max_theta));

%% Define prediction model
nlobj.Model.StateFcn = @(x, u) twsbr_discrete_dynamics(x, u, Ts, params);
nlobj.Model.IsContinuousTime = false;
nlobj.Model.OutputFcn = @(x, u) x;

%% Define cost function
Q_nmpc = diag([
    0,      ... x [ignore local position]
    50,     ... θ [moderate - stay reasonably upright]
    500,    ... ψ [VERY HIGH - heading is everything!]
    100,    ... dx [HIGH - velocity tracking]
    20,     ... dθ [moderate pitch rate penalty]
    100     ... dψ [high - smooth heading changes]
]);

R_nmpc = diag([0.1, 0.1]);

nlobj.Weights.OutputVariables = diag(Q_nmpc)';
nlobj.Weights.ManipulatedVariables = diag(R_nmpc)';
nlobj.Weights.ManipulatedVariablesRate = 0.1*diag(R_nmpc)';

fprintf('\nCost function weights:\n');
fprintf('  State weights Q (diagonal): ');
fprintf('%.1f ', diag(Q_nmpc));
fprintf('\n');

%% Validate
fprintf('\nValidating NMPC controller...\n');
validateFcns(nlobj, zeros(6,1), zeros(2,1));
fprintf('NMPC controller validated successfully!\n');

%% Define GLOBAL trajectory
fprintf('\n--- Global Trajectory Definition ---\n');

trajectory_type = 'figure8';  % 'circle', 'line', 'figure8', 'sine'

switch trajectory_type
    case 'sine'
        v_forward = 0.8;   % Velocity (m/s)
        A_wave = 3.0;      % Amplitude (m)
        omega_wave = 0.2;  % Frequency (rad/s)
        X_global_ref = @(t) v_forward * t;
        Y_global_ref = @(t) A_wave * sin(omega_wave * t);
        dX_global_ref = @(t) v_forward;
        dY_global_ref = @(t) A_wave * omega_wave * cos(omega_wave * t);
        fprintf('Sinusoidal trajectory:\n');
        fprintf('  Forward velocity: %.2f m/s\n', v_forward);
        fprintf('  Amplitude: %.1f m\n', A_wave);
        fprintf('  Frequency: %.2f rad/s\n', omega_wave);
        fprintf('  Wavelength: %.2f m\n', 2*pi*v_forward/omega_wave);
    case 'circle'
        R_circle = 8.0;    % Radius (m)
        omega = 0.15;      % Angular velocity (rad/s)
        X_global_ref = @(t) R_circle * cos(omega*t);
        Y_global_ref = @(t) R_circle * sin(omega*t);
        dX_global_ref = @(t) -R_circle * omega * sin(omega*t);
        dY_global_ref = @(t) R_circle * omega * cos(omega*t);
        fprintf('Circular trajectory: R = %.1f m, omega = %.2f rad/s\n', R_circle, omega);
        fprintf('Linear velocity: %.2f m/s\n', R_circle * omega);
    case 'line'
        v_line = 0.5;  % Velocity (m/s)
        X_global_ref     = @(t) v_line * t;
        Y_global_ref = @(t) 2.0;  % Constant Y
        dX_global_ref = @(t) v_line;
        dY_global_ref = @(t) 0;
        fprintf('Line trajectory: v = %.2f m/s, Y = %.1f m\n', v_line, 2.0);
    case 'figure8'
        A = 3.0;
        omega = 0.3;
        X_global_ref = @(t) A * sin(omega*t);
        Y_global_ref = @(t) A * sin(2*omega*t) / 2;
        dX_global_ref = @(t) A * omega * cos(omega*t);
        dY_global_ref = @(t) A * omega * cos(2*omega*t);
        fprintf('Figure-8 trajectory: A = %.1f m\n', A);
end

%% Initial conditions
x0 = 0;
theta0 = 0.05;
dx0 = 0;
dtheta0 = 0;
dpsi0 = 0;

% Set initial position and heading based on trajectory type
if strcmp(trajectory_type, 'sine')
    X_global_0 = 0.0;  % Start at origin
    Y_global_0 = 0.0;  % On the sine wave at t=0
    psi0 = 0;          % Initially facing East
elseif strcmp(trajectory_type, 'circle')
    X_global_0 = 8.0;  % Start on circle
    Y_global_0 = 0.0;
    psi0 = pi/2;       % Facing tangent (North)
elseif strcmp(trajectory_type, 'line')
    X_global_0 = 0.0;  % Start on the line
    Y_global_0 = 2.0;  % At the correct Y
    psi0 = 0;          % Facing East (along line)
else
    X_global_0 = 0;
    Y_global_0 = 0;
    psi0 = 0;
end

x_init = [x0; theta0; psi0; dx0; dtheta0; dpsi0];

fprintf('\n--- Initial Conditions ---\n');
fprintf('Initial global position: (%.2f, %.2f) m\n', X_global_0, Y_global_0);
fprintf('Initial heading: %.2f deg\n', rad2deg(psi0));

%% Simulation parameters
t_final = 30;
N_steps = round(t_final / Ts);

t_sim = (0:N_steps) * Ts;
x_hist = zeros(nx, N_steps+1);
u_hist = zeros(nu, N_steps);
X_global_hist = zeros(1, N_steps+1);
Y_global_hist = zeros(1, N_steps+1);
X_ref_global_hist = zeros(1, N_steps+1);
Y_ref_global_hist = zeros(1, N_steps+1);
x_ref_hist = zeros(ny, N_steps+1);

x_hist(:, 1) = x_init;
X_global_hist(1) = X_global_0;
Y_global_hist(1) = Y_global_0;

%% Run NMPC simulation
fprintf('\n--- Running Closed-Loop Simulation with NMPC ---\n');
fprintf('Simulating %.1f seconds (%d steps)...\n', t_final, N_steps);

u_prev = [0; 0];
h_wait = waitbar(0, 'Running NMPC simulation...');

for k = 1:N_steps
    if mod(k, 10) == 0
        waitbar(k/N_steps, h_wait, sprintf('Running NMPC simulation... %.1f\\%%', 100*k/N_steps));
    end

    % Current state
    x_current = x_hist(:, k);
    X_global_current = X_global_hist(k);
    Y_global_current = Y_global_hist(k);
    psi_current = x_current(3);

    % Desired global position at current time
    X_ref_global = X_global_ref(t_sim(k));
    Y_ref_global = Y_global_ref(t_sim(k));
    dX_ref_global = dX_global_ref(t_sim(k));
    dY_ref_global = dY_global_ref(t_sim(k));

    % Store for plotting
    X_ref_global_hist(k) = X_ref_global;
    Y_ref_global_hist(k) = Y_ref_global;

    % Desired heading: follow path tangent
    psi_ref = atan2(dY_ref_global, dX_ref_global);

    % Desired velocity: match path velocity
    v_ref_global = sqrt(dX_ref_global^2 + dY_ref_global^2);

    % Reference: [x, theta, psi, dx, dtheta, dpsi]
    x_ref = [0, 0, psi_ref, v_ref_global, 0, 0];
    x_ref_hist(:, k) = x_ref';

    % Compute optimal control
    [u_opt, ~] = nlmpcmove(nlobj, x_current, u_prev, x_ref, []);
    u_hist(:, k) = u_opt;

    % Simulate one step
    x_next = twsbr_discrete_dynamics(x_current, u_opt, Ts, params);

    % Check ground contact
    if abs(x_next(2)) > theta_limit
        fprintf('\nGround contact at t = %.3f s\n', t_sim(k+1));
        x_hist = x_hist(:, 1:k+1);
        u_hist = u_hist(:, 1:k);
        X_global_hist = X_global_hist(1:k+1);
        Y_global_hist = Y_global_hist(1:k+1);
        X_ref_global_hist = X_ref_global_hist(1:k+1);
        Y_ref_global_hist = Y_ref_global_hist(1:k+1);
        t_sim = t_sim(1:k+1);
        break;
    end

    x_hist(:, k+1) = x_next;

    % Update global position
    X_global_hist(k+1) = X_global_hist(k) + Ts * x_next(4) * cos(x_next(3));
    Y_global_hist(k+1) = Y_global_hist(k) + Ts * x_next(4) * sin(x_next(3));

    u_prev = u_opt;
end

close(h_wait);
fprintf('Simulation completed.\n');

% Final reference
X_ref_global_hist(end) = X_global_ref(t_sim(end));
Y_ref_global_hist(end) = Y_global_ref(t_sim(end));

%% Extract states
x = x_hist(1, :)';
theta = x_hist(2, :)';
psi = x_hist(3, :)';
dx = x_hist(4, :)';
dtheta = x_hist(5, :)';
dpsi = x_hist(6, :)';
t = t_sim';

psi_wrapped = mod(psi + pi, 2*pi) - pi;

X_global = X_global_hist';
Y_global = Y_global_hist';
X_ref_global = X_ref_global_hist';
Y_ref_global = Y_ref_global_hist';

error_X_global = X_global - X_ref_global;
error_Y_global = Y_global - Y_ref_global;
error_distance = sqrt(error_X_global.^2 + error_Y_global.^2);

%% Plotting
figure('Position', [100, 100, 1400, 900], 'Color', 'w');
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');

% Global X-Y trajectory
subplot(2, 3, [1 4]);
plot(X_global, Y_global, 'b-', 'LineWidth', 2.5);
hold on;
plot(X_ref_global, Y_ref_global, 'r--', 'LineWidth', 2);
plot(X_global(1), Y_global(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot(X_global(end), Y_global(end), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);
hold off;
xlabel('$X$ [m]', 'FontSize', 14);
ylabel('$Y$ [m]', 'FontSize', 14);
title('Global Path Following', 'FontSize', 16);
legend('Actual', 'Reference', 'Start', 'End', 'Location', 'best', 'FontSize', 12);
grid on;
axis equal;
set(gca, 'FontSize', 12);

% Tracking error
subplot(2, 3, 2);
plot(t, error_distance, 'k-', 'LineWidth', 2);
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('$e_{\mathrm{pos}}$ [m]', 'FontSize', 14);
title('Position Tracking Error', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);

% Pitch angle
subplot(2, 3, 3);
plot(t, rad2deg(theta), 'b-', 'LineWidth', 2);
hold on;
yline(rad2deg(max_theta), 'm--', 'LineWidth', 1.5);
yline(-rad2deg(max_theta), 'm--', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1);
hold off;
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('$\theta$ [$^\circ$]', 'FontSize', 14);
title('Pitch Angle', 'FontSize', 14);
legend('$\theta(t)$', 'Constraint', '', 'Target', 'Location', 'best', 'FontSize', 11);
grid on;
set(gca, 'FontSize', 12);

% Yaw angle
subplot(2, 3, 5);
plot(t, rad2deg(psi_wrapped), 'r-', 'LineWidth', 2);
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('$\psi$ [$^\circ$]', 'FontSize', 14);
title('Yaw Angle', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);

% Control inputs
subplot(2, 3, 6);
stairs(t(1:end-1), u_hist(1,:), 'b-', 'LineWidth', 2);
hold on;
stairs(t(1:end-1), u_hist(2,:), 'r-', 'LineWidth', 2);
hold off;
xlabel('$t$ [s]', 'FontSize', 14);
ylabel('$\tau$ [Nm]', 'FontSize', 14);
title('Control Torques', 'FontSize', 14);
legend('$\tau_L$', '$\tau_R$', 'Location', 'best', 'FontSize', 12);
grid on;
set(gca, 'FontSize', 12);

sgtitle('NMPC-Based Global Path Following', 'FontSize', 18, 'FontWeight', 'bold');

function x_next = twsbr_discrete_dynamics(x, u, Ts, params)
    dx_dt = twsbr_dynamics_continuous(x, u, params);

    % Euler step
    x_next = x + Ts * dx_dt;
end

function dx_dt = twsbr_dynamics_continuous(x, u, params)
    theta = x(2);
    dx = x(4);
    dtheta = x(5);
    dpsi = x(6);

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

    a11 = mb + 2*mw + 2*J/r^2;
    a12 = mb*l*cos(theta);
    a21 = a12;
    a22 = I2 + mb*l^2;
    a33 = I3 + 2*K + mw*d^2/2 + J*d^2/(2*r^2) - (I3 - I1 - mb*l^2)*(sin(theta))^2;

    M = [a11 a12 0; a21 a22 0; 0 0 a33];

    c12 = -mb*l*dtheta*sin(theta);
    c13 = -mb*l*dpsi*sin(theta);
    c23 = (I3 - I1 - mb*l^2)*dpsi*sin(theta)*cos(theta);
    c31 = mb*l*dpsi*sin(theta);
    c32 = -(I3-I1-mb*l^2)*dpsi*sin(theta)*cos(theta);
    c33 = -(I3-I1-mb*l^2)*dtheta*sin(theta)*cos(theta);

    C = [0 c12 c13; 0 0 c23; c31 c32 c33];

    d11 = 2*calpha/r^2;
    d12 = -2*calpha/r;
    d21 = d12;
    d22 = 2*calpha;
    d33 = calpha*d^2/(2*r^2);

    D = [d11 d12 0; d21 d22 0; 0 0 d33];

    b11 = 1/r;
    b12 = b11;
    b21 = -1;
    b22 = b21;
    b31 = -d / (2*r);
    b32 = -b31;

    B = [b11 b12; b21 b22; b31 b32];

    g2 = -mb*l*g*sin(theta);
    G = [0; g2; 0];

    qdot = [dx; dtheta; dpsi];
    qddot = M \ (B*u - C*qdot - D*qdot - G);

    dx_dt = [qdot; qddot];
end
