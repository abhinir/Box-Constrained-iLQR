function [cx,cu,cxx,cuu,cxu] = compute_cost_der(model, X_bar, U_bar, sigma, h)
% h = min(1e-5, min(abs(model.u_min - U_bar)/10, abs(U_bar - model.u_max)/10));


% h = 1e-6;
n = model.nx;
m = model.nu;
cx = zeros(n,1);
cu = zeros(m,1);
%cx = model.Q*X_bar;
%cu = model.R * U_bar - sigma*((1/(U_bar - model.u_min)) - (1/(model.u_max - U_bar)));
%cuu = model.R + sigma*((1/((U_bar - model.u_min)^2)) + (1/((model.u_max - U_bar)^2)));
%cxx = model.Q;
% cxx = zeros(n,n);
% cuu = zeros(m,m);
% cxu = zeros(n,m);
for i = 1:1:n
     e = zeros(n,1);
     e(i) = h;
     cx(i) = (model.cost(0,X_bar+e, U_bar,model,sigma) -...
         model.cost(0,X_bar-e, U_bar,model,sigma))/(2*h);
end

for i = 1:1:m
     e = zeros(m,1);
     e(i) = h;
     cu(i) = (model.cost(0,X_bar, U_bar+e,model,sigma) - ...
         model.cost(0,X_bar, U_bar-e,model,sigma))/(2*h);
end

cxx = zeros(n,n);
cxu = zeros(n,m);
cuu = zeros(m,m);

for i = 1:1:n
    for j = 1:1:n
        e1 = zeros(n,1);
        e1(i) = h;
        e2 = zeros(n,1);
        e2(j) = h;
        cxx(i,j) = (model.cost(0,X_bar+e1+e2, U_bar,model, sigma) - ...
                     model.cost(0,X_bar+e1-e2, U_bar,model, sigma) - ...
                     model.cost(0,X_bar-e1+e2, U_bar,model, sigma) + ...
                     model.cost(0,X_bar-e1-e2, U_bar,model, sigma))/(4*h^2);
    end
end

for i = 1:1:m
    for j = 1:1:m
        e1 = zeros(m,1);
        e1(i) = h;
        e2 = zeros(m,1);
        e2(j) = h;
        cuu(i,j) = (model.cost(0,X_bar, U_bar+e1+e2,model, sigma) - ...
                     model.cost(0,X_bar, U_bar+e1-e2,model, sigma) - ...
                     model.cost(0,X_bar, U_bar-e1+e2,model, sigma) + ...
                     model.cost(0,X_bar, U_bar-e1-e2,model, sigma))/(4*h^2);
    end
end

% for i = 1:1:n
%     for j = 1:1:m
%         e1 = zeros(n,1);
%         e1(i) = h;
%         e2 = zeros(m,1);
%         e2(j) = h;
%         cxu(i,j) = (model.cost(0,X_bar+e1, U_bar+e2,model, sigma) - ...
%                      model.cost(0,X_bar+e1, U_bar-e2,model, sigma) - ...
%                      model.cost(0,X_bar-e1, U_bar+e2,model, sigma) + ...
%                      model.cost(0,X_bar-e1, U_bar-e2,model, sigma))/(4*h^2);
%     end
% end

end