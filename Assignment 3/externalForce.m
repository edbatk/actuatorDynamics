function [F] = externalForce(t,X,c,p,C)
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
%
% Kevin Green 2021

% Calculate random disturbance force
% rng('default')
% random = rand(1,8);
% disturbances = c.amp*random;

F = 0;

% if t < 2 && t >= 1
%     F = C(1);
% elseif t < 3 && t >= 2
%     F = C(2);
% elseif t < 4 && t >= 3
%     F = C(3);
% elseif t < 5 && t >= 4
%     F = C(4);
% elseif t < 6 && t >= 5
%     F = C(5);
% elseif t < 7 && t >= 6
%     F = C(6);
% elseif t < 8 && t >= 7
%     F = C(7);
% elseif t < 9 && t >= 8
%     F = C(8);   
% elseif t < 10 && t >= 9
%     F = C(9);
% else
%     F = C(10);
% end

