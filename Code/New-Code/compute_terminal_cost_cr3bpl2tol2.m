function t_cost = compute_terminal_cost_cr3bpl2tol2(state_err, Qf)
t_cost = 0.5*state_err'*Qf*state_err;
end