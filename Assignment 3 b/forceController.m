function [F, y, dy] = forceController(t,X,c,p,trajectory_fun)

[des_pos, des_vel, des_accel] = trajectory_fun(t);
F = c.Kp.*(des_pos - X(3)) + c.Kd.*(des_vel - X(4)) + des_accel*p.ma;

end