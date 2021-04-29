function paddleAnimation(p,t,X,c,wallFun,exportVideo,playbackRate)
% Paddle Animation 
% Input
%   p: Simulation constants
%   t: Simulation time vector
%   X: Simulation state vector
%   exportVideo: Should the video be exported? (True/False)
% Output
%   An animation/File
% By Kevin Green 2021

% FPS for playback and video export
FPS = 60; % If your computer cannot plot in realtime lower this.

% For SE3
addpath(fullfile(pwd,'..', 'groupTheory'))
% For CubeClass and SpringClass
addpath(fullfile(pwd,'..', 'visualization'))

% Create objects
% ceilingObj = CubeClass([2,2,0.2]);
block_h = 0.25;
actuatorObj = CubeClass([1, block_h]);
wallObj = CubeClass([5, block_h]);
springObj = SpringClass;

% Create a figure handle
h.figure = figure;
%This sets figure dimension which will dictate video dimensions
h.figure.Position(3:4) = [1280 720];
movegui(h.figure)

% Put the shapes into a plot
% ceilingObj.plot
actuatorObj.colors =     [.1 .7 1;
                          .1 .7 1;
                          .1 .7 1;
                          .1 .7 1;
                          .1 .7 1;
                          .1 .7 1;
                          .1 .7 1;
                          .1 .7 1];
                         
actuatorObj.plot
wallObj.plot
springObj.plot

% Figure properties
view(2)
title('Simulation')
xlabel('x Position (m)')
ylabel('y Position (m)')
zlabel('z Position (m)')
% These commands set the aspect ratio of the figure so x scale = y scale
% "Children(1)" selects the axes that contains the animation objects
h.figure.Children(1).DataAspectRatioMode = 'manual';
h.figure.Children(1).DataAspectRatio = [1 1 1];

% exportVideo = true;

% Setup videowriter object
if exportVideo  
   v = VideoWriter('C:\Users\edwar\Documents\School\ROB542\actuatorDynamics\Assignment 2 b b\Video.mp4', 'MPEG-4');
%    v = VideoWriter('puckAnimation.avi');
   v.FrameRate = FPS;
   
   open(v)
end

% Iterate over state data
tic;
for t_plt = t(1):playbackRate*1.0/FPS:t(end)
    
    x_state = interp1(t',X',t_plt);
    xa_pos = x_state(1);
    
    wall_pos = wallFun(t_plt);

    % Set axis limits (These will respect the aspect ratio set above)
    h.figure.Children(1).XLim = [-5, 5];
    h.figure.Children(1).YLim = [-5, 5];
    h.figure.Children(1).ZLim = [-1.0, 1.0];
    
    % Set the wall position
    wallObj.resetFrame
    wallObj.globalMove(SE3([0, wall_pos, 0]));
    
    % Set the actuator position
    actuatorObj.resetFrame
    actuatorObj.globalMove(SE3([0, xa_pos - block_h, 0]));
    
    % Spring position
    springObj.resetFrame
    springObj.updateState(SE3([0, xa_pos - block_h, 0, 0, 0, pi/2]), wall_pos + block_h - xa_pos);
    
    % Update data
    actuatorObj.updatePlotData
    wallObj.updatePlotData
    springObj.updatePlotData
    
    if exportVideo %Draw as fast as possible for video export
        drawnow
        frame = getframe(h.figure);
        writeVideo(v,frame);
    else % pause until 1/FPS of a second has passed then draw
        while( toc < 1.0/FPS)
            pause(0.002)
        end
        drawnow
        tic;
    end % if exportvideo
end % t_plt it = ...

end % paddleAnimation