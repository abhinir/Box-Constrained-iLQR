clc;
clear;
close all;
%% Initialization

model = model_register('CR3BPL2toL2');

num_outer_loop_iter = 1000;
maxIte  = 100;
u_merged = [];
x_merged = [];
sigma_merged = [];
sigma = 1e10*ones(1, model.nu);



red_factor = 0.6;
beta = (1/0.95);
eps_barr = 1e-3;

%%

u_guess = 0.00*ones(model.nu, model.horizon);


feedback = zeros(model.nu, model.nx, model.horizon);

tic
for i = 1:1:num_outer_loop_iter
    
    [x_nom, u_nom, K, lambda] =...
        Box_ILQR(model, model.X0, model.Xg, u_guess, maxIte, sigma, false);
    
    
    if norm(sigma, inf) < eps_barr || red_factor_state>1 || red_factor_con>1
        break;
    end
    if any(u_nom > model.u_max, 'all') || any(u_nom < model.u_min, 'all')
        fprintf('Control Constraint Violated\n')
        sigma = sigma./red_factor;
        red_factor = red_factor*beta;
        sigma = sigma.*red_factor;
        model.alpha = model.alpha*0.5;
        continue;
    else
        u_guess = u_nom;
        u_merged = [u_merged;u_nom];
        x_merged = [x_merged;x_nom];
        sigma_merged = [sigma_merged;sigma];
        sigma = sigma.*red_factor;
        feedback = K;
    end

end
toc
