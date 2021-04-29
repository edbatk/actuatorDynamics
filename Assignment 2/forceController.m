function [F] = forceController(t,X,c,p,desDefl,wallFun)
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

[pos_wall, vel_wall, accel_wall] = wallFun(t);
actualDefl = pos_wall - X(1); 

F_ff = p.ma*accel_wall;

% Calculate control force
F = c.Kp.*(actualDefl - desDefl) + F_ff - p.k*desDefl;





end