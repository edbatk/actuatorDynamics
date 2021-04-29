function [F] = forceController(p,t,X,c)

y_des = 0;
dy_des = 0;
y = X(1);
dy = X(2);
    
F_ff = (p.ma + p.mt + p.ml)*9.81;

% Calculate control force
F = c.Kp.*(y_des - y) + ...
    c.Kd.*(dy_des - dy) + F_ff;

% disp(F)
end