clc;
close all;
%%
% Initialize parameters
numIterations = 100;

K = feedback;
% K = zeros(size(feedback));
x_nom = x_merged(end-5:end,:);
u_nom = u_merged(end-2:end,:);

% Dimensions
nx = size(x_nom, 1);
nu = size(u_nom, 1);
N  = model.horizon;

% Preallocate Monte Carlo trajectories
x_noisy = zeros(nx, N + 1, numIterations);
u_noisy = zeros(nu, N,     numIterations);
cost    = zeros(1, numIterations);

% Start a parallel pool if one is not already running
if isempty(gcp('nocreate'))
    parpool;
end

parfor i = 1:numIterations

    cost_i = 0;

    % Local state and control histories for this Monte Carlo run
    x_i = zeros(nx, N + 1);
    u_i_history = zeros(nu, N);

    % Reproducible random-number stream for this iteration
    rng((i - 1) * model.horizon);

    % Initial state
    x_i(:,1) = x_nom(:,1);

    % Initial feedback control
    u_i = u_nom(:,1) + ...
          K(:,:,1) * (x_i(:,1) - x_nom(:,1));

    % Add control noise
    sigma = abs(u_i) / 30;
    u_i = u_i + sigma .* randn(size(u_i));

    % Apply control bounds
    u_i = min(max(u_i, model.u_min(:)), model.u_max(:));

    % Store noisy control
    u_i_history(:,1) = u_i;

    % Accumulate control cost
    cost_i = cost_i + norm(u_i, 2) * model.dt;

    % Propagate the trajectory
    for j = 1:N-1

        % State propagation using control u(:,j)
        x_i(:,j+1) = model.state_prop( ...
            j * model.dt, ...
            x_i(:,j), ...
            u_i, ...
            model);

        % Reproducible noise for this time step
        rng((i - 1) * model.horizon + j);

        measurement_noise = [ ...
            1e-4 .* randn(3,1);
            1e-6 .* randn(3,1)];

        process_noise = [ ...
            (1/30) .* randn(1,1) .* u_nom(1,j);
            (1/30) .* randn(1,1) .* u_nom(2,j);
            (1/30) .* randn(1,1) .* u_nom(3,j)];

        % Feedback control for time step j+1
        u_i = u_nom(:,j+1) + ...
              K(:,:,j+1) * ...
              (x_i(:,j+1) - x_nom(:,j+1));

        % Add control noise
        sigma = abs(u_i) / 30;
        u_i = u_i + sigma .* randn(size(u_i));

        % Apply control saturation
        u_i = min(max(u_i, model.u_min(:)), model.u_max(:));

        % Store noisy saturated control
        u_i_history(:,j+1) = u_i;

        % Accumulate control cost
        cost_i = cost_i + norm(u_i, 2) * model.dt;
    end

    % Final propagation using u(:,N)
    x_i(:,N+1) = model.state_prop( ...
        N * model.dt, ...
        x_i(:,N), ...
        u_i, ...
        model);

    % Assign local results to sliced outputs
    x_noisy(:,:,i) = x_i;
    u_noisy(:,:,i) = u_i_history;
    cost(i) = cost_i;
end

%%
error = zeros(6, numIterations);
error_norm_dis = zeros(1, numIterations);
error_norm_vel = zeros(1, numIterations);
for i = 1:1:numIterations
    error(:,i) = x_noisy(1:6, end, i) - model.Xg;
    error(:,i) = [error(1:3,i)*3.84405e8;error(4:6,i)*1023.23281];
    error_norm_dis(i) = norm(error(1:3,i), 2);
    error_norm_vel(i) = norm(error(4:6,i), 2);
end

%%
% for i = 10:1:10
%     for j = 1:1:6
%         figure(j)
%         plot(x_noisy(j,:,i), '--k')
%         hold on
%         grid on
%     end
% end
% 
% for k =1:1:6
%     figure(k)
%     plot(x_nom(k,:), LineWidth=2, color = 'r')
%     legend('Perturbed','Nominal')
% end

% for i = 21:1:22
%     for j = 1:1:6
%         figure(j)
%         plot(x_noisy(j,:,i),'--k')
%         hold on
%         grid on
%     end
% end

%%
%% All noisy control realizations

nu = size(u_noisy,1);
N  = size(u_noisy,2);

t_u = (0:N-1)*model.dt;

% Control constraints
u_max =  0.05/sqrt(3);
u_min = -0.05/sqrt(3);

controlLabels = {'u_x','u_y','u_z'};

for k = 1:nu

    figure('Color','w');

    tiledlayout(1,1, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    ax = nexttile;
    hold(ax,'on');

    % N x number of Monte Carlo simulations
    u_all_k = squeeze(u_noisy(k,:,:));

    % Plot all noisy realizations
    h_noisy = plot(t_u,u_all_k, ...
        'Color',[0.75,0.75,0.75], ...
        'LineWidth',0.5);

    set(h_noisy,'HandleVisibility','off');

    % Dummy line for noisy-realization legend entry
    plot(nan,nan, ...
        'Color',[0.75,0.75,0.75], ...
        'LineWidth',1.5, ...
        'DisplayName','Noisy realizations');

    % Nominal control
    plot(t_u,u_nom(k,1:N), ...
        'k-', ...
        'LineWidth',0.2, ...
        'DisplayName','Nominal control');

    % Upper and lower control bounds
    yline(u_max,'k:', ...
        'LineWidth',1.5, ...
        'HandleVisibility','off');

    yline(u_min,'k:', ...
        'LineWidth',1.5, ...
        'HandleVisibility','off');

    xlabel('Time (TU)', ...
        'Interpreter','latex', ...
        'FontSize',13);

    ylabel(['$' controlLabels{k} '$'], ...
        'Interpreter','latex', ...
        'FontSize',13);

    legend('Interpreter','latex', ...
        'Location','best', ...
        'Box','off');

    xlim([0,13.0789122308]);

    grid on;
    box on;

    ax.FontSize = 12;
    ax.LineWidth = 1.1;
    ax.TickLabelInterpreter = 'latex';
end