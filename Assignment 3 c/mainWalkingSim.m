% Simulate, plot, and animate a puck bouncing between a wall and a moving
% paddle. This is to illustrate hybrid simulation using event functions
% Author:  Kevin Green 2021

clear global;

X0 = [ 0, ...       % x position (m)
       1.5, ...     % x velocity (m/s)
       1, ...       % y position (m)
       0];          % y velocity (m/s)

% Simulation System Constants
p.mw = 1.5;   % mass of the walker (kg)
p.k = 170;  % stiffness (N/m)
p.c = 2;    % damping (Ns/m)
p.l0 = 1;   % rest length of spring legs (m)
p.x_t1 = 0;
p.x_t2 = 0.5;
p.y_t1 = 0;
p.y_t2 = 0;
p.tdAngle = pi/3; %%%
p.step = 0.75;

z.steps = 0;

c.Kp = 250;          % position error feedback gain (N/m)
c.Kd = 50;           % velocity error feedback gain (N/(m/s))

td_control = @(t,X) walkingController(t,X,p,c); 
     
% % Simulate the system
[t_vec,X_vec] = walkingSim(X0,p,z,td_control);

% Plot the position of the puck, paddle and ceiling
figure;
subplot(2,1,1) 
plot(t_vec,X_vec(1,:),'--');
title("X Positions");
xlabel('Time (s)')
ylabel('Position (m)')

subplot(2,1,2)
plot(t_vec,X_vec(3,:),'--');
title("Y Positions");
xlabel('Time (s)')
ylabel('Position (m)')

% Animate the mass
exportVideo = true;
playbackRate = 1;
walkingAnimation(p,t_vec,X_vec,exportVideo,playbackRate);