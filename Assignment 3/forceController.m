function [F] = forceController(t,X,c,p,desDefl)
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

actualDefl = (X(3) - X(1)); 

% Calculate control force
% F = -c.Kp*(desDefl - actualDefl) + c.Kd*(X(4) - X(2)) + c.Kpull*(0 - X(1)) - p.k*desDefl;
F = c.Kp*(desDefl - actualDefl) - p.k*desDefl;
% F = -desDefl*p.k;
% F = 0;

end