% Simulate, plot, and animate a puck bouncing between a wall and a moving
% paddle. This is to illustrate hybrid simulation using event functions
% Author:  Kevin Green 2021

clear;

% Simulation System Constants
p.ma = 1; % mass (kg) of the actuator
p.mm = 2;
p.k = 5; % Stiffness (N/m)
p.c = 5; % Damping (Ns/m)
p.l0 = 2;
p.ml = 0.1;
p.mt = .1;
p.m2 = p.ml + p.mt;
p.e = 0.9;

c.Kp = 200;          % position error feedback gain (N/m)
c.Kd = 25;           % velocity error feedback gain (N/(m/s))
c.amp = 1;
c.freq = 5;

X0 = [0, ...        % actuator position (m)
      0];                     
   
controller_fun = @(t,X) forceController(t,X,c,p); 
     
% % Simulate the system
[t_vec,X_vec] = dynamicsSim(X0,p,controller_fun);

% Plot the position of the puck, paddle and ceiling
figure;
subplot(2,1,1) 
plot(t_vec,X_vec(1,:),'--');
hold on
plot(t_vec,X_vec(3,:) + p.l0,'--');
title("Positions");
xlabel('Time (s)')
ylabel('Position (m)')
legend('Actuator Position','Wall Position')

% Animate the mass
exportVideo = false;
playbackRate = 2;
Animation(p,t_vec,X_vec,c,exportVideo,playbackRate);