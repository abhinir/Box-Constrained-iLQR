function [Vx, Vxx] = compute_terminal_cost_der_docking(state_err, Qf)
Vx = Qf*state_err;
Vxx = Qf;
end