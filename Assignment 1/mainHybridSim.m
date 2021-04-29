% Simulate, plot, and animate a puck bouncing between a wall and a moving
% paddle. This is to illustrate hybrid simulation using event functions
% Author:  Kevin Green 2021

clear;

X0 = [ 0, ...       % actuator position (m)
       0, ...       % actuator velocity (m/s)
       0, ...       % toe position (m) DATUMMM
       0, ...       % toe velocity (m/s)
       3, ...       % load position (m)
       0];          % load velocity (m/s)

% Simulation System Constants
p.ma = 1; % mass (kg) of the actuator
p.k = 100; % Stiffness (N/m)
p.c = 1; % Damping (Ns/m)
p.l0 = 2;
p.ml = 0.1;
p.mt = .1;
p.m2 = p.ml + p.mt;

c.Kp = 250;          % position error feedback gain (N/m)
c.Kd = 50;           % velocity error feedback gain (N/(m/s))
c.amp = 1;
c.freq = 3;

controller_fun = @(t,X) PDTrajectoryController(t,X,c,'Position'); 
     
% % Simulate the system
[t_vec1,X_vec1] = paddleSim2(X0,p,controller_fun);
P.ml = 1;
[t_vec2,X_vec2] = paddleSim2(X0,p,controller_fun);
p.ml = 3;
[t_vec3,X_vec3] = paddleSim2(X0,p,controller_fun);
p.ml = 5;
[t_vec4,X_vec4] = paddleSim2(X0,p,controller_fun);

% Plot the position of the puck, paddle and ceiling
figure;
subplot(2,2,1) 
plot(t_vec1,X_vec1(1,:),'--');
hold on
plot(t_vec1,X_vec1(3,:)+p.l0,'--');
plot(t_vec1,X_vec1(5,:)+p.l0,'--');
title("Positions: Load Mass = 0.1 kg");
xlabel('Time (s)')
ylabel('Position (m)')
legend('Actuator Position','Toe Position','Load Position')

subplot(2,2,2) 
plot(t_vec1,X_vec2(1,:),'--');
hold on
plot(t_vec1,X_vec2(3,:)+p.l0,'--');
plot(t_vec1,X_vec2(5,:)+p.l0,'--');
title("Positions: Load Mass = 1 kg");
xlabel('Time (s)')
ylabel('Position (m)')
legend('Actuator Position','Toe Position','Load Position')

subplot(2,2,3) 
plot(t_vec1,X_vec3(1,:),'--');
hold on
plot(t_vec1,X_vec3(3,:)+p.l0,'--');
plot(t_vec1,X_vec3(5,:)+p.l0,'--');
title("Positions: Load Mass = 3 kg");
xlabel('Time (s)')
ylabel('Position (m)')
legend('Actuator Position','Toe Position','Load Position')

subplot(2,2,4) 
plot(t_vec1,X_vec4(1,:),'--');
hold on
plot(t_vec1,X_vec4(3,:)+p.l0,'--');
plot(t_vec1,X_vec4(5,:)+p.l0,'--');
title("Positions: Load Mass = 5 kg");
xlabel('Time (s)')
ylabel('Position (m)')
legend('Actuator Position','Toe Position','Load Position')

% figure
% % Plot the Velocity of the mass
% subplot(2,1,2)
% plot(t_vec,X_vec(2,:),'-');
% hold on;
% plot(t_vec,X_vec(4,:),'-');
% plot(t_vec,X_vec(6,:),'-');
% title('Velocities')
% xlabel('Time (s)')
% ylabel('Velocity (m/s)')
% legend('Actuator Velocity', 'Toe Velocity', 'Load Velocity')

% Animate the mass
exportVideo = true;
playbackRate = 2;
paddleAnimation(p,t_vec4,X_vec4,exportVideo,playbackRate);