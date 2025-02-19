function [F] = forceController(t,X,c,p,wallFun)
% This is a PD + feed forward controller function to track a circle.
% It supports being called in batch mode for retreiving commands after
% simulation (n dimensional input).
% Given:
%   t: current time
%       1xn
%   X: current state [position_x position_y velocity_x velocity_y]
%       4xn or 4x1
%   c.Kp:  position error feedback gain (N/m)
%   c.Kd:  velocity error feedback gain (N/(m/s))
%   c.Kff: feedforward acceleration gain (N/(m/s^2))
%   c.center: center point of the circle [position_x position_y] (m)
%       2x1
%   c.radius: radius of the circle to trace (m)
%   c.frequency: angular velocity of the circle (rad/sec)
%
% Returns:
%   F: Force Command
%       2xn
%   r_des: Desired Position from Trajectory
%       2xn
%   dr_des: Desired Velocity from Trajectory
%       2xn
%   ddr_des: Desired Acceleration from Trajectory
%       2xn
%
% Kevin Green 2021


%Get the desired position, velocity and acceleration of the circular path


% Calculate control force
% F = c.amp.*sin(t.*c.freq);

y_des = c.amp*sin(t.*c.freq);
dy_des = c.amp.*c.freq.*cos(t.*c.freq);
ddy_des = -c.amp.*c.freq.^2*sin(t.*c.freq);
y = X(1);
dy = X(2);

[p_w, v_w] = wallFun(t);
% F_ff = (p.k*(p_w - y_des) + p.c*(v_w - dy_des))*p.ma;
F_ff = (p.k*(y_des - p_w) + p.c*(dy_des - v_w) + ddy_des)*p.ma;
% F_ff = 0;

% Calculate control force
F = c.Kp.*(y_des - y) + ...
    c.Kd.*(dy_des - dy) + F_ff;

end