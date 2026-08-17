function [Vx, Vxx] = compute_terminal_cost_der_cr3bpl2tol2(state_err, Qf)
Vx = Qf*state_err;
Vxx = Qf;
end