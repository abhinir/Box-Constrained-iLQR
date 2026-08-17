function [Fx, Fu] = ilqr_der(model, X_bar, U_bar)

h = 1e-5;
n = model.nx;
m = model.nu;
Fx = zeros(n,n);
Fu = zeros(n,m);
% X_next = forward_euler(0,X_bar,U_bar,model);
for i = 1:1:n
     e = zeros(n,1);
     e(i) = h;
     Fx(:,i) = (forward_euler(0,X_bar+e, U_bar,model) -...
         forward_euler(0,X_bar-e, U_bar,model))/(2*h);
end

for i = 1:1:m
     e = zeros(m,1);
     e(i) = h;
     Fu(:,i) = (forward_euler(0,X_bar, U_bar+e,model) - ...
         forward_euler(0,X_bar, U_bar-e,model))/(2*h);
end



end

% function [A,B] = ilqr_der(model, X_bar, U_bar)
% 
% X_next = forward_euler(0,X_bar,U_bar,model);
% 
% n = 2*(model.nx + model.nu);
% del_x_next_p = zeros(model.nx,n);
% del_x_next_m = zeros(model.nx,n);
% del_x_next = zeros(model.nx,n);
% del_x = zeros(model.nx,n);
% del_u = zeros(model.nu,n);
% Y = [del_x;del_u];
% 
% for i = 1:1:n
%     del_x(:,i) = normrnd(0,1e-3,[model.nx,1]);
%     del_u(:,i) = normrnd(0,1e-3,[model.nu,1]);
%     Y(:,i) = [del_x(:,i);del_u(:,i)];
%     del_x_next_p(:,i) = ...
%         forward_euler(0,X_bar+del_x(:,i), U_bar+del_u(:,i),model) - X_next;
%     del_x_next_m(:,i) = ...
%         forward_euler(0,X_bar-del_x(:,i), U_bar-del_u(:,i),model) - X_next;
%     del_x_next(:,i) = (del_x_next_p(:,i) - del_x_next_m(:,i))./2;
% 
% end
% P = del_x_next*(Y')*inv(Y*Y');
% A = P(:,1:model.nx);
% B = P(:,model.nx+1:end);
% end