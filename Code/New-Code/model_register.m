function [model] = model_register(modelName)


if strcmp(modelName, 'pendulum')
    model.name = 'pendulum';
    model.m = 0.5;
    model.L = 0.5;
    model.g = 9.81;

    model.nl_ode = @pendulum_nl_ode;
    model.state_prop = @pendulum_nl_state_prop;
    
    model.horizon = 5/model.dt;
    model.u_min = -1;
    model.u_max = 1;
    model.dt = 0.01;
    model.nx = 2;
    model.nu = 1;
    model.alpha = 1;
    model.alpha_floor = 1e-8;
    
    model.X0 = [0*pi/180;0];  %theta (rad), thetadot (rad/s)
    model.Xg = [180*pi/180;0];
    model.Q = 3 * eye(model.nx) * model.dt;
    model.R = 3 * eye(model.nu) * model.dt;
    model.Qf = 30*eye(model.nx);
    model.alpha = 1;

elseif  strcmp(modelName, 'cartpole')
    model.name = 'cartpole';
    model.M = 1;
    model.m = 0.01;
    model.L = 0.6;
    model.g = 9.81;

    model.dt = 0.01;
    model.nx = 4;
    model.nu = 1;
    
    model.alpha = 1;
    model.Xg = [0;0;0*pi/180;0]; %x, xdot, theta(rad), thetadot(rad/s)
    model.X0 = [0;0;180*pi/180;0];% pole bottom is pi
    model.R = 10*model.dt*eye(model.nu);
    model.Q = 10*model.dt*eye(model.nx);
    model.Qf = 10000*eye(model.nx);
    
    model.nl_ode = @cartpole_nl_ode;
    model.state_prop = @cartpole_nl_state_prop;
    
    model.horizon = 10/model.dt; %time horizon of the finite-horizon OCP
    
elseif strcmp(modelName, 'Acrobot')
    model.name = 'Acrobot';
    model.nx = 4;
    model.nu = 1;
    model.dt = 0.01;
    model.alpha = 1;
    model.u_max = 5;
    model.u_min = -5;
    model.Xg = [pi;0;0;0];
    model.X0 = [0;0;0;0];
    model.l1 = 1;
    model.l2 = 1;
    model.m1 = 1;
    model.m2 = 1;
    model.g = 9.81;
    model.Q = 500*model.dt*diag([1, 1, 1, 1]);
    model.R = 10*model.dt;
    model.Qf = 50000*diag([1, 1, 1, 1]);
    model.horizon = 10/model.dt;
    model.nl_ode = @acrobot_nl_ode;
    model.state_prop = @acrobot_nl_state_prop;


elseif strcmp(modelName, 'Attitude')
    model.name = 'Attitude';
    model.nx = 6;
    model.nu = 3;

    model.dt = 1;
    model.horizon = 3000/model.dt;

    model.alpha = 1;
    model.u_max = 1e-3*ones(model.nu, 1);
    model.u_min = -1e-3*ones(model.nu, 1);
    
    model.I = diag([140, 120, 130]);

    model.Xg = [pi;-pi/4;-pi/2;0;0;0];
    model.X0 = [0;0;0;0;0;0];

    model.Q = 5*model.dt*diag([1, 1, 1, 1, 1, 1]);
    model.R = 5*model.dt*diag([1, 1, 1]);
    model.Qf = 50000*diag([1, 1, 1, 1, 1, 1]);
    
    model.nl_ode = @attitude_nl_ode;
    model.state_prop = @attitude_nl_state_prop;
    model.cost = @compute_cost_attitude;
    model.terminal_cost = @compute_terminal_cost_attitude;
    model.terminal_cost_der = @compute_terminal_cost_der_attitude;

elseif strcmp(modelName, 'Orbit_transfer')
    model.name = 'Orbit_transfer';
    model.nx = 3;
    model.nu = 2;

    model.dt = 6*pi/1000;
    model.horizon = 1000;

    model.alpha = 1;
    model.u_max = [1;pi/3];
    model.u_min = [0;-pi/3];

    model.a = 2e-3;
    model.mu = 1;

    model.Xg = [1.1;0;1];
    model.X0 = [1.08;0;sqrt(model.mu/1.08)];

    model.Q = 5e-4*model.dt*diag([1, 1, 0]);
    model.R = 5e-3*model.dt*diag([1, 1e-6]);
    model.Qf = 1e9*diag([1, 1e-3, 0]);

    model.nl_ode = @orbit_transfer_nl_ode;
    model.state_prop = @orbit_transfer_nl_state_prop;
    
    model.cost = @compute_cost_orbit_transfer;
    model.terminal_cost = @compute_terminal_cost_orbit_transfer;
    model.terminal_cost_der = @compute_terminal_cost_der_orbit_transfer;

elseif strcmp(modelName, 'Docking')
    model.name = 'Docking';

    model.nx = 13;
    model.nu = 3;

    model.dt = 1;
    model.horizon = 1000;

    model.alpha = 1;

    model.u_max = [5/sqrt(3);5/sqrt(3);5/sqrt(3)];
    model.u_min = -[5/sqrt(3);5/sqrt(3);5/sqrt(3)];

    model.Isp = 220;
    model.g0 = 9.80665;
    model.mue = 3.986004418e14;

    model.Xg = [0;0;0;0;0;0;0;0;0;0;0;0;0];
    model.X0 = [100;15;-10;-0.1;0;0;500;6778136.3;0;0;0;7668.558;0];

    model.Q = 5*model.dt*diag([1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0]);
    model.R = 5*model.dt*diag([1, 1, 1]);
    model.Qf = 5e4*diag([1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0]);

    model.nl_ode = @docking_nl_ode;
    model.state_prop = @docking_nl_state_prop;

    model.cost = @compute_cost_docking;
    model.terminal_cost = @compute_terminal_cost_docking;
    model.terminal_cost_der = @compute_terminal_cost_der_docking;



elseif strcmp(modelName, 'CR3BPL2toL2')
    model.name = 'CR3BPL2toL2';

    model.nx = 6;
    model.nu = 3;

    model.dt = 13.0789122308048/10000;
    model.horizon = 10000;

    model.alpha = 1;

    model.u_max = 1*[0.05/sqrt(3);0.05/sqrt(3);0.05/sqrt(3)];
    model.u_min = -1*[0.05/sqrt(3);0.05/sqrt(3);0.05/sqrt(3)];

    
    model.mu = 1.21506683e-2;

    model.X0 = [
     1.180640625697826;...
     0;...
    -0.0162093101977567;...
     0;...
    -0.157362892253317;...
     0];
   

    model.Xg = [
        1.13039436638415;...
        0;...
        -0.176209310197757;...
        0;...
        -0.225445576494454;...
        0];
   
    model.Q = 0*model.dt*diag([1, 1, 1, 1, 1, 1]);
    model.R = 5*model.dt*diag([1, 1, 1]);
    model.Qf = 50e8*diag([1, 1, 1, 1e-4, 1e-4, 1e-4]);

    model.nl_ode = @cr3bpl2tol2_nl_ode;
    model.state_prop = @cr3bpl2tol2_nl_state_prop;

    model.cost = @compute_cost_cr3bpl2tol2;
    model.terminal_cost = @compute_terminal_cost_cr3bpl2tol2;
    model.terminal_cost_der = @compute_terminal_cost_der_cr3bpl2tol2;

end

end
