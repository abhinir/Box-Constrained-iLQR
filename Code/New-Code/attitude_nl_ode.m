function [state_dot] = attitude_nl_ode(t, state, U, model, dist)
    % --- Model Parameters ---
    I = model.I;
    
    % --- Unpack State ---
    phi = state(1);
    th  = state(2);
    w   = state(4:6);   % angular velocity vector [w1; w2; w3]

    % Avoid Singularity
    c_th  = cos(th);
    if abs(c_th) < 1e-10 
        c_th = 1e-10; 
    end

    % --- Transformation Matrix ---
    T = [1 sin(phi)*tan(th)  cos(phi)*tan(th);
         0 cos(phi)         -sin(phi);
         0 sin(phi)/c_th     cos(phi)/c_th];
    
    if dist
        % rng(ceil(t))
        d1 = normrnd(0,1e-5,[3,1]);
        rng(1)
        freq_sin = normrnd(0,1e-4, [3,1]);
        freq_cos = normrnd(0,1e-4, [3,1]);
        amp_sin = normrnd(0,1e-6, [3,1]);
        amp_cos = normrnd(0,1e-6, [3,1]);
        d2 = amp_sin.*sin(freq_sin.*t) + amp_cos.*cos(freq_cos.*t);
        d = d1 + d2;
    else
        d = zeros(3,1);
    end
    % --- Kinematics and Dynamics ---
    angle_dot = T * w;
   
    
    w_dot = I \ (U - cross(w, I*w) + d);

    % --- Assign to State Derivative ---
    state_dot = [angle_dot; w_dot];
end