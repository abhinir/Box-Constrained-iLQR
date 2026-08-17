function [x_nom, u_nom, K, Vx] =...
    Box_ILQR(model, x0, xg, u_nom, maxIte, sigma, hessian)


horizon = model.horizon;
QT = model.Qf;
x_nom = zeros(model.nx,horizon+1); 
x_nom(:,1) = x0;
dx = zeros(model.nx,horizon);
Vxx = zeros(model.nx,model.nx,horizon+1); Vxx(:,:,horizon+1) = QT; %CTG - hessian
Vx = zeros(model.nx,horizon+1);% CTG - jacobian
K = zeros(model.nu,model.nx,horizon); % feedback gain for control acting on delta x
Kv = zeros(model.nu,model.nx,horizon);
Ku = zeros(model.nu,model.nu,horizon);
Quu = zeros(model.nu,model.nu,horizon);
kt  = zeros(model.nu,horizon);% feedback gain for control constant term
At = zeros(model.nx,model.nx,horizon);
Bt = zeros(model.nx,model.nu,horizon);
cxt = zeros(model.nx,horizon);
cut = zeros(model.nu,horizon);
cxxt = zeros(model.nx,model.nx,horizon);
cuut = zeros(model.nu,model.nu,horizon);
cxut = zeros(model.nx,model.nu,horizon);
h = 1e-6;

criteria = true;

der = struct();

alpha = model.alpha; %step size
alpha_floor = 1e-8;
lr = [];

iter = 1;

change_cost_crit = 1; %parameter used to check if the current solution should be kept or ignored.
cost0 = -1;

%% forward pass

while iter <= maxIte && criteria
    forward_flag = true;
    constraint_violation = false;
    s = 1;
    while forward_flag

        cost_new = 0;
        x_new(:,1) = x0;
        u_new = u_nom;

        for i=1:horizon
            dx(:,i) = x_new(:,i) - x_nom(:,i);
            
            u_new(:,i) = u_nom(:,i) + K(:,:,i)*dx(:,i) + ...
                                    alpha*kt(:,i);
            for j=1:1:model.nu
                if model.u_max(j) < u_new(j,i)   || u_new(j,i) <  model.u_min(j)
                    maxIte = iter;
                    criteria = false;
                    constraint_violation = true;
                end
                s = min(s, (model.u_max(j) - u_new(j,i)));
                s = min(s, (u_new(j,i) - model.u_min(j)));
            end
            % if state_constraints_docking(x_new(:,i), 0)
            %     maxIte = iter;
            %     criteria = false;
            %     constraint_violation = true;
            % end
            state_err = compute_state_error(x_new(:,i), model.Xg, model.name);

            cost_new = cost_new + model.cost(0,state_err,u_new(:,i),model,sigma);
            x_new(:,i+1) = model.state_prop((i-1)*model.dt, x_new(:,i), u_new(:,i), model);
        end

        state_err = compute_state_error(x_new(:,horizon+1), model.Xg, model.name);

        cost_new = cost_new + model.terminal_cost(state_err, QT);
        
        [Vx(:,horizon+1),Vxx(:,:,horizon+1)] = model.terminal_cost_der(state_err, QT);

        if iter > 1
            change_cost_crit = (cost_new - cost(iter - 1))/delta_J;
        end

        % if (change_cost_crit > (1 - eps) && change_cost_crit<(1 + eps)) %refer IROS2012 Todorov 
        if (change_cost_crit > 0 || iter == maxIte) || constraint_violation
        %if (change_cost_crit <= 0 || alpha < 10^-5)
            fprintf('change_cost_crit = %d\n', change_cost_crit);
            % if (change_cost_crit < (1 - eps) || change_cost_crit>(1 + eps))
            %     count = count + 1;
            % end
            forward_flag = false;
            cost(iter) = cost_new;
            x_nom = x_new;
            u_nom = u_new;
            state_err = x_nom(:,horizon+1) -  xg;

            [Vx(:,horizon+1),Vxx(:,:,horizon+1)] = model.terminal_cost_der(state_err, QT);
            

%             if alpha<0.0005
%                 alpha=0.0005;
%             end
        else
            % delta_J = delta_J./(-alpha + (alpha^2)/2);
            % alpha = 0.99*alpha;
            % delta_J = delta_J * (-alpha + (alpha^2)/2);
            % if alpha<1e-20
            %     break;
            % end
            % fprintf('alpha = %f \n', alpha);

            % fprintf('change_cost_crit = %d\n', change_cost_crit);
            % % if (change_cost_crit < (1 - eps) || change_cost_crit>(1 + eps))
            % %     count = count + 1;
            % % end
            % forward_flag = false;
            % cost(iter) = cost_new;
            % x_nom = x_new;
            % u_nom = u_new;
            % state_err = compute_state_error(x_nom(:,horizon+1), model.Xg, model.name);
            % 
            % vk(:,horizon+1) = QT*(state_err);
            if alpha > alpha_floor
                delta_J1 = delta_J1/alpha;
                delta_J2 = delta_J2*2/(alpha^2);
                alpha = 0.9*alpha;
                delta_J1 = delta_J1*alpha;
                delta_J2 = delta_J2*(alpha^2)/2;
                delta_J = delta_J1 + delta_J2;
                % fprintf('Reducing learning rate: %d\n', alpha)
            else
                alpha = alpha_floor;
                fprintf('change_cost_crit = %d\n', change_cost_crit);
                forward_flag = false;
                cost(iter) = cost_new;
                x_nom = x_new;
                u_nom = u_new;
                state_err = x_nom(:,horizon+1) - xg;

                [Vx(:,horizon+1),Vxx(:,:,horizon+1)] = model.terminal_cost_der(state_err, QT);
                break;
            end
        end

    end

    x_traj_ite_ilqr(:,:,iter) = x_nom;
    u_traj_ite_ilqr(:,:,iter) = u_nom;
    
    lr = [lr, alpha];
    state_err = x_nom(:,end) - xg;

    state_error_norm = norm(state_err);
    fprintf('iter = %d; state_error_norm=%d; cost=%d; lr=%d \n',iter,...
                state_error_norm,cost_new, alpha);
    %[iter state_error_norm cost_new]

    %% backward pass
    if criteria == false
        continue;
    end
    if (s > 0) && (s < 1e-5)
        h = s / 10;
        disp(s);
    end
    parfor (i = 1:horizon, horizon)
        % find perturbation matrices
        if ~hessian
            [A, B] = ilqr_der(model, x_nom(:,i), u_nom(:,i));
        else
            [A,B,Fxx,Fux,Fuu] = ddp_der(model, x_nom(:,i), u_nom(:,i));
        end
        [cx,cu,cxx,cuu,cxu] = compute_cost_der(model,x_nom(:,i) - xg, u_nom(:,i),sigma, h);
        fx(:,:,i) = A;
        fu(:,:,i) = B;
        if hessian
            fxx(:,:,:,i) = Fxx;
            fux(:,:,:,i) = Fux;
            fuu(:,:,:,i) = Fuu;
        end
        cxt(:,i) = cx
        cut(:,i) = cu;
        cxxt(:,:,i) = cxx;
        cuut(:,:,i) = cuu;
        cxut(:,:,i) = cxu;
    end
    
    delta_J1 = 0;
    delta_J2 = 0;
    % delta_J=0; %expected total cost reduction
    
    for i=horizon:-1:1

        
        % gains
        
        % Quu(:,:,i) = Bt(:,:,i)'*Sk(:,:,i+1)*Bt(:,:,i) + cuut(:,:,i);
        
        % if min(eig(Quu(:,:,i))) <= 0
        %     disp('Quu is not positive definite\n')
        %     min(eig(Quu(:,:,i)))
        %     pause;
        % end
        Hv_xx = zeros(model.nx, model.nx);
        Hv_ux = zeros(model.nu, model.nx);
        Hv_uu = zeros(model.nu, model.nu);
        if hessian
            Hv_xx = tensor_multiply(Vx(:,i+1), fxx(:,:,:,i));
            Hv_ux = tensor_multiply(Vx(:,i+1), fux(:,:,:,i));
            Hv_uu = tensor_multiply(Vx(:,i+1), fuu(:,:,:,i));
        end

        Qx = cxt(:,i) + fx(:,:,i)'*Vx(:,i+1);
        Qu = cut(:,i) + fu(:,:,i)'*Vx(:,i+1);
        Qxx = cxxt(:,:,i) + fx(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i) + Hv_xx;
        Qux = cxut(:,:,i)' + fu(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i) + Hv_ux;
        Quu = cuut(:,:,i) + fu(:,:,i)'*Vxx(:,:,i+1)*fu(:,:,i) + Hv_uu;
        
         if min(eig(Quu)) < 0
            fprintf('Quu is not positive definite: %.12f\n',min(eig(Quu)))
            % criteria = false;
            % break
            min_eig = min(eig(Quu));
            Quu_reg = (-1*min_eig + 1)*eye(size(Quu)) + Quu;
         else
             Quu_reg = Quu;
         end 
        % min_eig = min(eig(Quu));
        % Quu_reg = sign(min_eig)*5*min_eig*eye(size(Quu)) + Quu;
        % Quu_reg = 10*min_eig*eye(size(Quu)) + Quu;
        kt(:,i) = -pinv(Quu_reg)*Qu;
        K(:,:,i) = -pinv(Quu_reg)*Qux;
        
    
        Vx(:,i) = Qx + K(:,:,i)'*Quu*kt(:,i) + K(:,:,i)'*Qu + Qux'*kt(:,i);
        Vxx(:,:,i) = Qxx + K(:,:,i)'*Quu*K(:,:,i) + K(:,:,i)'*Qux + Qux'*K(:,:,i);

        % delta_J = delta_J  - (alpha^2/2)*Qu'*inv(Quu)*Qu;
        delta_J1 = delta_J1 + alpha*kt(:,i)'*Qu;
        delta_J2 = delta_J2 + alpha^2/2*kt(:,i)'*Quu*kt(:,i);
    end
    delta_J = delta_J1 + delta_J2;
    if cost0 == -1
        cost0 = cost(1);
    end
    % 
    % if iter >= 2
    %     conv_rate(idx) = abs((cost(iter-1)-cost(iter)));%/cost0);
    %     idx = idx + 1;
    % end
    % if idx > length(conv_rate)
    %     idx = 1;
    % end
    p = norm(kt,inf);
    
     

    %rate_conv_diff = abs(conv_rate(1) - conv_rate(2)) + abs(conv_rate(2) - conv_rate(3));
    % cost_change = sum(conv_rate);
   
    % if ((abs(cost_change) < 0.00001) || iter == maxIte || p<1e-6)
    if iter>=1
        if (iter == maxIte || p<1e-4)
            criteria = false;
        end
    end
    if iter>1
        if abs((cost_new - cost(iter-1))/(cost_new*alpha)) < 1e-4
            criteria = false;
        end
    end
    iter = iter + 1;
    % if (norm(kt,inf) < 1e-8 || iter == maxIte)
    %     criteria = false;
    % end

end

der.fx = At;
der.fu = Bt;
der.fxx = [];
der.fuu = [];
der.fux = [];
der.cx = cxt;
der.cu = cut;
der.cxx = cxxt;
der.cuu = cuut;
der.cxu = cxut;


% count
end

