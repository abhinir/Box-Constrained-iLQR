function [state_n] = attitude_nl_state_prop(t, state, U, model, dist)

t_span = [(t-1)*model.dt, t*model.dt];

if nargin < 5
    dist = false;
end

% [temp, X_out] = ode45(@(t,y) pendulum_nl_ode(t,y,U,model), t_span, state);
%X_out(end,1) = atan2(sin(X_out(end,1)), cos(X_out(end,1)));

% state_n = X_out(end,:);
X_out = forward_euler_attitude(t, state, U, model, dist);
% state_n = X_out(end,:);
state_n = X_out;
end