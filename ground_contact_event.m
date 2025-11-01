function [value, isterminal, direction] = ground_contact_event(t, state, theta_limit)
% GROUND_CONTACT_EVENT Event function to detect when robot body hits ground
%
% This function detects when the pitch angle exceeds the physical limit
% where the robot body would contact the ground.
%
% Inputs:
%   t - current time
%   state - state vector [x; theta; psi; dx; dtheta; dpsi]
%   theta_limit - maximum allowable pitch angle (rad)
%
% Outputs:
%   value - goes to zero when event occurs
%   isterminal - 1 to stop integration, 0 to continue
%   direction - 0 for all crossings

% Extract pitch angle
theta = state(2);

% Event occurs when |theta| reaches theta_limit
% We want to detect when: theta_limit - |theta| = 0
value = theta_limit - abs(theta);

% Stop integration when ground contact occurs
isterminal = 1;

% Detect zero crossing in either direction
direction = 0;

end
