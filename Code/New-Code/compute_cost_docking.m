function cost = compute_cost_docking(t,x,u,model, sigma)
sigma_l = sigma(1:3);
sigma_u = sigma(1:3);
sigma_x = sigma(4);

cost = 0.5*x'*model.Q*x + 0.5*u'*model.R*u...
    - model.dt*sigma_l*log(u - model.u_min) - model.dt*sigma_u*log(model.u_max - u)...
    - model.dt*sigma_x*log(x(2) + x(1)) - model.dt*sigma_x*log(x(1) - x(2))...
    - model.dt*sigma_x*log(x(3) + x(1)) - model.dt*sigma_x*log(x(1) - x(3));
end