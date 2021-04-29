% Simulate, plot, and animate a puck bouncing between a wall and a moving
% paddle. This is to illustrate hybrid simulation using event functions
% Author:  Kevin Green 2021

clear;

X0 = [ 0, ...       % actuator position (m)
       0, ...       % actuator velocity (m/s)
       0, ...       % toe position (m)
       0];          % toe velocity (m/s)

% Simulation System Constants
p.ma = 1; % mass (kg) of the actuator
p.k = 100; % Stiffness (N/m)
p.c = 1; % Damping (Ns/m)
p.l0 = 1;
p.mt = 1;

c.Kp = 10;          % position error feedback gain (N/m)
c.Kd = 25;           % velocity error feedback gain (N/(m/s))
c.Kpull = 25;
c.amp = 1;
c.freq = 3;


desForce = 100;
desDefl = 2;
controller_fun = @(t,X) forceController(t,X,c,p,desDefl);
  
% % Simulate the system
[t_vec,X_vec] = Sim(X0,p,controller_fun);

spring_force = (p.k*(X_vec(3,:) - X_vec(1,:)) + p.c*(X_vec(4,:) - X_vec(2,:)));

% Plot the position of the puck, paddle and ceiling
figure;
subplot(2,1,1) 
plot(t_vec,X_vec(1,:),'--');
hold on
plot(t_vec,X_vec(3,:)+p.l0,'--');
title("Positions: Load Mass = 0.1 kg");
xlabel('Time (s)')
ylabel('Position (m)')
legend('Actuator Position','Toe Position','Load Position')
% 
subplot(2,1,2) 
plot(t_vec,ones(size(t_vec)) * desForce,'--');
hold on
title("System Forces");
plot(t_vec,spring_force,'--');
xlabel('Time (s)')
ylabel('Force (N)')
legend('Desired Force', 'Actual Force')

% Animate the mass
exportVideo = false;
playbackRate = 2;
Animation(p,t_vec,X_vec,exportVideo,playbackRate);