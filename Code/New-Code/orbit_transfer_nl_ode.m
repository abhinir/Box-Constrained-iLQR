function [state_dot] = orbit_transfer_nl_ode(~, state, U, model)
% --- Model Parameters ---
mu = model.mu;

% --- Unpack State ---
r = state(1);
u = state(2);
v = state(3);

%
a = model.a;
s = U(1);
phi = U(2);
%
rdot = u;
udot = (v.^2)./r - mu./(r.^2) + s*a*sin(phi);
vdot = -(u.*v)./r + s*a*cos(phi);

state_dot = [rdot;udot;vdot];

end