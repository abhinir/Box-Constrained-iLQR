function [state_n] = docking_nl_state_prop(t, state, U, model, dist)

t_span = [(t-1)*model.dt, t*model.dt];

if nargin < 5
    dist = false;
end

X_out = forward_euler(t, state, U, model, dist);

state_n = X_out;
end