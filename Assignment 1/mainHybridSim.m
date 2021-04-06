% Simulate, plot, and animate a puck bouncing between a wall and a moving
% paddle. This is to illustrate hybrid simulation using event functions
% Author:  Kevin Green 2021

% Simulation System Constants
p.ma  = 1; % mass (kg) of the paddle
p.F = -1; % Constant external force (N) gravity
p.e = 0.9; % Coefficient of restitution (0,1]
p.d_wall = 10.0; % Distance to the wall (m)
p.k = 1; % Stiffness (N/m)
p.c = .1; % Damping (Ns/m)
p.l0 = 1;
p.ml = 1;
p.mt = 1;

% Initial state conditions
X0 = [ 0, ...        % actuator position (m)
       0, ...        % actuator velocity (m/s)
       p.l0, ...     % toe position (m)
       0, ...        % toe velocity (m/s)
       p.l0 + 1, ... % load position (m)
       0];          % load velocity (m/s)

% Paddle Trajectory Constants
c.freq = 0; % frequency of paddle oscillation (rad/s)
c.amp = 0.2; % Amplitude of paddle motion (m)

% Bind the trajectory constants to paddle motion function
paddleFun = @(t) paddleMotion(t,c);

% Simulate the system
[t_vec,X_vec] = paddleSim(X0,p,paddleFun);

% Plot the position of the puck, paddle and ceiling
figure;
subplot(2,1,1) 
plot(t_vec,X_vec(1,:),'-');
hold on
plot(t_vec,paddleFun(t_vec));
plot(t_vec, p.d_wall*ones(size(t_vec)));
xlabel('Time (s)')
ylabel('Position (m)')
legend('Ball Position','Paddle Position','Ceiling Position')
% Plot the Velocity of the mass
subplot(2,1,2)
plot(t_vec,X_vec(2,:),'-');
title('Mass Response')
xlabel('Time (s)')
ylabel('Velocity (m)')

% Animate the mass
% exportVideo = false;
% playbackRate = 2;
% paddleAnimation(p,t_vec,X_vec,paddleFun,exportVideo,playbackRate);