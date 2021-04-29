function [pos] = desiredTrajectory(t,c)
% This is a PD + feed forward controller function to track a circle.
% It supports being called in batch mode for retreiving commands after
% simulation (n dimensional input).
% Given:
%   t: current time
%       1xn
%   c.Kp:  position error feedback gain (N/m)
%   c.Kd:  velocity error feedback gain (N/(m/s))
%
% Returns:
%   pos: Current Position of the paddle
%       2xn
%   vel: Current Velocity of the paddle
%       2xn
%


% pos = c.amp*(sin(2*pi*0.05*t+2*pi*rand).*c.freq);

pos = cos(2*pi*0.3*t+2*pi*rand) + randn(1)*sin(t);
% pos = c.amp.*sin(t.*c.freq);

end
