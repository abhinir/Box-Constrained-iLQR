% function [Fx, Fu, Fxx, Fux, Fuu] = ddp_der(model, X_bar, U_bar)
% 
% h = 1e-3;
% n = model.nx;
% m = model.nu;
% Fx = zeros(n,n);
% Fu = zeros(n,m);
% X_next = forward_euler(0,X_bar,U_bar,model);
% for i = 1:1:n
%      e = zeros(n,1);
%      e(i) = h;
%      Fx(:,i) = (forward_euler(0,X_bar+e, U_bar,model) -...
%          forward_euler(0,X_bar-e, U_bar,model))/(2*h);
% end
% 
% for i = 1:1:m
%      e = zeros(m,1);
%      e(i) = h;
%      Fu(:,i) = (forward_euler(0,X_bar, U_bar+e,model) - ...
%          forward_euler(0,X_bar, U_bar-e,model))/(2*h);
% end
% 
% 
% Fxx = zeros(n,n,n);
% Fuu = zeros(n,m,m);
% Fux = zeros(n,m,n);
% 
% for i = 1:n
%     ei = zeros(n,1);  ei(i) = h;
% 
%     % diagonal entry
%     Fxx(:,i,i) = ( forward_euler(0,X_bar+ei,   U_bar,model) ...
%                  - 2*X_next ...
%                  + forward_euler(0,X_bar-ei,   U_bar,model) ) / h^2;
% 
%     for j = i+1:n
%         ej = zeros(n,1);  ej(j) = h;
% 
%         Fxx(:,i,j) = ( forward_euler(0,X_bar+ei+ej, U_bar,model) ...
%                      - forward_euler(0,X_bar+ei-ej, U_bar,model) ...
%                      - forward_euler(0,X_bar-ei+ej, U_bar,model) ...
%                      + forward_euler(0,X_bar-ei-ej, U_bar,model) ) / (4*h^2);
% 
%         Fxx(:,j,i) = Fxx(:,i,j);         % exploit symmetry
%     end
% end
% 
% 
% parfor i = 1:m
%     ei = zeros(m,1);  ei(i) = h;
%     for j = 1:n
%         ej = zeros(n,1);  ej(j) = h;
% 
%         Fux(:,i,j) = ( forward_euler(0,X_bar+ej, U_bar+ei, model) ...
%                      - forward_euler(0,X_bar-ej, U_bar+ei, model) ...
%                      - forward_euler(0,X_bar+ej, U_bar-ei, model) ...
%                      + forward_euler(0,X_bar-ej, U_bar-ei, model) ) / (4*h^2);
%     end
% end
% 
% 
% for i = 1:m
%     ei = zeros(m,1);  ei(i) = h;
% 
%     % diagonal entry
%     Fuu(:,i,i) = ( forward_euler(0,X_bar,   U_bar+ei,model) ...
%                  - 2*X_next ...
%                  + forward_euler(0,X_bar,   U_bar-ei,model) ) / h^2;
% 
%     for j = i+1:m
%         ej = zeros(m,1);  ej(j) = h;
% 
%         Fuu(:,i,j) = ( forward_euler(0,X_bar, U_bar+ei+ej,model) ...
%                      - forward_euler(0,X_bar, U_bar+ei-ej,model) ...
%                      - forward_euler(0,X_bar, U_bar-ei+ej,model) ...
%                      + forward_euler(0,X_bar, U_bar-ei-ej,model) ) / (4*h^2);
% 
%         Fuu(:,j,i) = Fuu(:,i,j);         % exploit symmetry
%     end
% end
% 
% 
% end



function [Fx, Fu, Fxx, Fux, Fuu] = ddp_der(model, X_bar, U_bar)

h = 1e-3;
n = model.nx;
m = model.nu;
Fx = zeros(n,n);
Fu = zeros(n,m);
X_next = forward_euler(0,X_bar,U_bar,model);
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


Fxx = zeros(n,n,n);
Fuu = zeros(n,m,m);
Fux = zeros(n,m,n);

for i = 1:n
    ei = zeros(n,1);  ei(i) = h;

    % diagonal entry
    Fxx(:,i,i) = ( forward_euler(0,X_bar+ei,   U_bar,model) ...
                 - 2*X_next ...
                 + forward_euler(0,X_bar-ei,   U_bar,model) ) / h^2;

    for j = i+1:n
        ej = zeros(n,1);  ej(j) = h;

        Fxx(:,i,j) = ( forward_euler(0,X_bar+ei+ej, U_bar,model) ...
                     - forward_euler(0,X_bar+ei-ej, U_bar,model) ...
                     - forward_euler(0,X_bar-ei+ej, U_bar,model) ...
                     + forward_euler(0,X_bar-ei-ej, U_bar,model) ) / (4*h^2);

        Fxx(:,j,i) = Fxx(:,i,j);         % exploit symmetry
    end
end


parfor i = 1:m
    ei = zeros(m,1);  ei(i) = h;
    for j = 1:n
        ej = zeros(n,1);  ej(j) = h;

        Fux(:,i,j) = ( forward_euler(0,X_bar+ej, U_bar+ei, model) ...
                     - forward_euler(0,X_bar-ej, U_bar+ei, model) ...
                     - forward_euler(0,X_bar+ej, U_bar-ei, model) ...
                     + forward_euler(0,X_bar-ej, U_bar-ei, model) ) / (4*h^2);
    end
end


for i = 1:m
    ei = zeros(m,1);  ei(i) = h;

    % diagonal entry
    Fuu(:,i,i) = ( forward_euler(0,X_bar,   U_bar+ei,model) ...
                 - 2*X_next ...
                 + forward_euler(0,X_bar,   U_bar-ei,model) ) / h^2;

    for j = i+1:m
        ej = zeros(m,1);  ej(j) = h;

        Fuu(:,i,j) = ( forward_euler(0,X_bar, U_bar+ei+ej,model) ...
                     - forward_euler(0,X_bar, U_bar+ei-ej,model) ...
                     - forward_euler(0,X_bar, U_bar-ei+ej,model) ...
                     + forward_euler(0,X_bar, U_bar-ei-ej,model) ) / (4*h^2);

        Fuu(:,j,i) = Fuu(:,i,j);         % exploit symmetry
    end
end


end