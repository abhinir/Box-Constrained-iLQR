function cost = compute_cost_attitude(t,x,u,model, sigma)
sigma_l = sigma;
sigma_u = sigma;
% sigma_u = sigma(1);
% sigma_x = sigma(2);
cost = 0.5*x'*model.Q*x + 0.5*u'*model.R*u...
    - model.dt*sigma_l*log(u - model.u_min) - model.dt*sigma_u*log(model.u_max - u);
% - sigma_x*model.dt*log(x(1) + 0.2) - sigma_x*model.dt*log(0.2 - x(1));

% cost = u(1)...
%     - sigma_l*log(u(1) - model.u_min) - sigma_u*log(model.u_max - u(1));
% disp(cost)
end