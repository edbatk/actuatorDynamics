function [F, y, dy] = PDTrajectoryController(t,X,c,type)
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
if type == 'Position'
    y_des = 0;
    dy_des = 0;
    y = X(1);
    dy = X(2);
    
end

if type == 'Sinusoid'
    y_des = c.amp*sin(t.*c.freq);
    dy_des = c.amp.*c.freq.*cos(t.*c.freq);
    y = X(1);
    dy = X(2);
end

% Calculate control force
F = c.Kp.*(y_des - y) + ...
    c.Kd.*(dy_des - dy);

if F > 1000
    F = 1000;
end

end