function [state_dot] = docking_nl_ode(t, state, U, model, dist)

if nargin<5
    dist = zeros(3,1);
else
    dist = normrnd(0, 0.1, [3, 1]);
end
% --- Model Parameters ---
I = model.Isp;
g = model.g0;
mu = model.mue;
% --- Unpacking states ---
r_dot = state(11:13);
m_dot = -norm(U, 2)./(I*g);
rho_dot = state(4:6);
r_norm = norm(state(8:10), 2);
r_rho_norm = norm(state(8:10) + state(1:3), 2);
v_dot = -(mu/(r_norm^3)).*state(8:10);
rho_ddot = -mu*(state(8:10) + state(1:3))/(r_rho_norm^3) +...
    (1/state(7)).*U +...
    (mu/(r_norm^3)).*state(8:10);

% --- Assign to State Derivative ---
state_dot = [rho_dot; rho_ddot; m_dot; r_dot; v_dot];
end