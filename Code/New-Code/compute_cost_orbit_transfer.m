function cost = compute_cost_orbit_transfer(t,x,u,model, sigma)
sigma_l = sigma;
sigma_u = sigma;
% sigma_u = sigma(1);
% sigma_x = sigma(2);
% cost = 0.5*x'*model.Q*x + 0.5*u'*model.R*u...
%     - model.dt*sigma_l*log(u - model.u_min) - model.dt*sigma_u*log(model.u_max - u);
cost = 1*model.dt.*(sqrt(u(1).^2 + (1e-12)^2) - 1e-12) + 0.5*u(2)*model.R(2,2)*u(2)...
    - model.dt*sigma_l*log(u - model.u_min) - model.dt*sigma_u*log(model.u_max - u);
end