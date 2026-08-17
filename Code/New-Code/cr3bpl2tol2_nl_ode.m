function [state_dot] = cr3bpl2tol2_nl_ode(~, state, U, model, dist)

if nargin<5
    dist = zeros(3,1);
end
% --- Model Parameters ---
mu = model.mu;

% --- Unpack State ---
x = state(1);
y = state(2);
z = state(3);
xdot = state(4);
ydot = state(5);
zdot = state(6);

% --- Define variables ---
r1 = sqrt((x + mu)^2 + y^2 + z^2);
r2 = sqrt((x + mu - 1)^2 + y^2 + z^2);

%
r_ddot = zeros(3,1);
r_ddot(1) = 2*ydot + x - (1 - mu)*(x + mu)/(r1^3)...
    - mu*(x - 1 + mu)/(r2^3) + U(1);

r_ddot(2) = -2*xdot + y - (1 - mu)*y/(r1^3)...
    - mu*y/(r2^3) + U(2);

r_ddot(3) = -(1 - mu)*z/(r1^3) - mu*z/(r2^3)...
    + U(3);


state_dot = [xdot;ydot;zdot;r_ddot];

end