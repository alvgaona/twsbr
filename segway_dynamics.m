function dstate = segway_dynamics(t, state, params, control_func)
% SEGWAY_DYNAMICS Computes the dynamics of a two-wheeled inverted pendulum
%
% Inputs:
%   t - current time
%   state - state vector [x; theta; psi; dx; dtheta; dpsi]
%           x: forward position, theta: pitch angle, psi: yaw angle
%   params - structure containing robot parameters
%   control_func - function handle for control input: u = control_func(t, state)
%
% Output:
%   dstate - time derivative of state vector

% Extract state variables
x = state(1);
theta = state(2);
psi = state(3);
dx = state(4);
dtheta = state(5);
dpsi = state(6);

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

% Get control input
u = control_func(t, state);

% Velocity vector
qdot = [dx; dtheta; dpsi];

% Equation of motion: M*qddot + C*qdot + D*qdot + G = B*u
% Solve for qddot: qddot = M\(B*u - C*qdot - D*qdot - G)
qddot = M \ (B*u - C*qdot - D*qdot - G);

% Construct derivative of state vector
dstate = [
    dx;           % d/dt(x) = dx
    dtheta;       % d/dt(theta) = dtheta
    dpsi;         % d/dt(psi) = dpsi
    qddot(1);     % d/dt(dx) = ddx
    qddot(2);     % d/dt(dtheta) = ddtheta
    qddot(3);     % d/dt(dpsi) = ddpsi
];

end
