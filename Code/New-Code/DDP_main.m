clc;
% clear;
close all;
%% Initialization
% load("results_att_con.mat");
load("results_cr3bp.mat")
model = model_register('CR3BPL2toL2');

num_outer_loop_iter = 1;
maxIte  = 1;
% u_merged = [];
% x_merged = [];
% sigma_merged = [];
% sigma = 1e10*ones(1, model.nu);



red_factor_con = 0.8;
red_factor_state = 0.5;
beta = (1/0.99);
eps_barr = 1e-6;
model.alpha = 0.2;
%%

% u_guess = 0.00*ones(model.nu, model.horizon);

u_guess = u_merged(358:360,:);
sigma = sigma_merged(120, :);

feedback = zeros(model.nu, model.nx, model.horizon);

tic
for i = 1:1:num_outer_loop_iter
    
    [x_nom, u_nom, K, lambda] =...
        Box_ILQR(model, model.X0, model.Xg, u_guess, maxIte, sigma, false);
    
    
    if norm(sigma, inf) < eps_barr || red_factor_state>1 || red_factor_con>1
        break;
    end
    if any(u_nom > model.u_max, 'all') || any(u_nom < model.u_min, 'all') %max(u_nom(1,:)) > model.u_max || min(u_nom(1,:)) < model.u_min
        fprintf('Control Constraint Violated\n')
        sigma = sigma./red_factor_con;
        red_factor_con = red_factor_con*beta;
        sigma = sigma.*red_factor_con;
        model.alpha = model.alpha*0.5;
        continue;
    % elseif state_constraints_docking(x_nom, model.horizon) %max(u_nom(1,:)) > model.u_max || min(u_nom(1,:)) < model.u_min
    %     fprintf('State Constraint Violated\n')
    %     sigma(4) = sigma(4)./red_factor_state;
    %     red_factor_state = red_factor_state*beta;
    %     sigma(4) = sigma(4).*red_factor_state;
    %     model.alpha = model.alpha*0.8;
    %     continue;
    else
        u_guess = u_nom;
        u_merged = [u_merged;u_nom];
        x_merged = [x_merged;x_nom];
        sigma_merged = [sigma_merged;sigma];
        sigma = sigma.*red_factor_con;
        % sigma(4) = sigma(4).*red_factor_state;
        feedback = K;
    end

end
toc
