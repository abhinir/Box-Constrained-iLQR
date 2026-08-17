function [state_err] = compute_state_error(x, x_bar, modelName)

state_err = x - x_bar;

end