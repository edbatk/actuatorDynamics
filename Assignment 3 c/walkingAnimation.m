function paddleAnimation(p,t,X,exportVideo,playbackRate)
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
actuatorObj = CubeClass([block_h, block_h]);
groundObj = CubeClass([100, 10, -0.1]);
springObj1 = SpringClass;
springObj2 = SpringClass;

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
springObj1.plot
springObj2.plot
groundObj.colors =       [0.8 .8 .8;
                          0.8 .8 .8;
                          0.8 .8 .8;
                          0.8 .8 .8;
                          0.8 .8 .8;
                          0.8 .8 .8;
                          0.8 .8 .8;
                          0.8 .8 .8];
groundObj.plot 

% Set the ceiling position, but offset it up because of the radius of
% the puck and half the side length of the cube. The position refers to the
% center of the cube.
% ceilingObj.globalMove(SE3([0 p.d_wall 0]));
% ceilingObj.updatePlotData

groundObj.globalMove(SE3([0,-5,0]));
groundObj.updatePlotData

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
   v = VideoWriter('C:\Users\edwar\Documents\School\ROB542\actuatorDynamics\Assignment 3 c\Video.mp4', 'MPEG-4');
%    v = VideoWriter('puckAnimation.avi');
   v.FrameRate = FPS;
   
   open(v)
end

% Iterate over state data
tic;

i_step = 1;

for t_plt = t(1):playbackRate*1.0/FPS:t(end)
    
    x_state = interp1(t',X',t_plt);
    xx_pos = x_state(1);
    xy_pos = x_state(3);

    % Set axis limits (These will respect the aspect ratio set above)
    h.figure.Children(1).XLim = [-1, 15];
    h.figure.Children(1).YLim = [-2, 6];
    h.figure.Children(1).ZLim = [-1.0, 1.0];
    
    % Set the actuator position
    actuatorObj.resetFrame
    actuatorObj.globalMove(SE3([xx_pos, xy_pos, .2]));
    
%     x_t1 = getGlobalStep;
%     x_t2 = x_t1 + 0.5;
    
    time = getGlobalTimes;
    step = getGlobalSteps;
    
    step_length = length(step);
    
    if step_length >= 1        
        if i_step <= step_length
            if t_plt < time(1)
                x_t1 = 0;
            elseif t_plt >= time(i_step)
                x_t1 = step(i_step);
                i_step = i_step + 1;
            end
        end
    else
        x_t1 = 0;
    end    
   
   
    x_t2 = x_t1 + 0.5;
%     disp(x_t1)
    
%     theta1 = atan((xy_pos - p.y_t1)/(p.x_t1 - xx_pos));
%     theta2 = atan((xy_pos - p.y_t2)/(p.x_t2 - xx_pos));
%     l1 = sqrt((xx_pos - p.x_t1).^2 + (xy_pos - p.y_t1).^2);
%     l2 = sqrt((xx_pos - p.x_t2).^2 + (xy_pos - p.y_t2).^2);
%     theta1 = atan((xy_pos - p.y_t1)/(x_t1 - xx_pos));
%     theta2 = atan((xy_pos - p.y_t2)/(x_t2 - xx_pos));
    theta1 = atan2((xy_pos - p.y_t1),(x_t1 - xx_pos));
    theta2 = atan2((xy_pos - p.y_t2),(x_t2 - xx_pos));
    l1 = sqrt((xx_pos - x_t1).^2 + (xy_pos - p.y_t1).^2);
    l2 = sqrt((xx_pos - x_t2).^2 + (xy_pos - p.y_t2).^2);
    if l1 > p.l0
        l1 = p.l0;
    end
    if l2 > p.l0
        l2 = p.l0;
    end
    
%     disp(theta2)
    
    % Spring position
    springObj1.resetFrame
    springObj1.updateState(SE3([xx_pos, xy_pos, 0.1, 0, 0, -theta1]), l1);
    
    springObj2.resetFrame
    springObj2.updateState(SE3([xx_pos, xy_pos, 0.1, 0, 0, -theta2]), l2);
    
    
    
    % Update data
    actuatorObj.updatePlotData
    springObj1.updatePlotData
    springObj2.updatePlotData
    
    
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
