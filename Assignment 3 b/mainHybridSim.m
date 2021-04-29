% Simulate, plot, and animate a puck bouncing between a wall and a moving
% paddle. This is to illustrate hybrid simulation using event functions
% Author:  Kevin Green 2021

clear;

X0 = [ 0, ...       % actuator position (m)
       0, ...       % actuator velocity (m/s)
       0, ...       % toe position (m)
       0];

% Simulation System Constants
p.ma = 1; % mass (kg) of the actuator
p.k = 100; % Stiffness (N/m)
p.c = 10; % Damping (Ns/m)
p.l0 = 2;
p.ml = 0.1;
p.mt = .1;
p.m2 = p.ml + p.mt;

c.Kp = 500;          % position error feedback gain (N/m)
c.Kd = 10;           % velocity error feedback gain (N/(m/s))
c.amp = 1;
c.freq = 3;

c.toolAmp = 10;
c.toolFreq = 100;

trajectory_fun = @(t,X) desiredTrajectory(t,c,p);
controller_fun = @(t,X) forceController(t,X,c,p,trajectory_fun);
external_fun   = @(t,X) extForce(t,X,c,p);
     
% % Simulate the system
[t_vec,X_vec] = dynamicsSim(X0,p,controller_fun,external_fun);

[des_traj, des_vel, des_accel] = trajectory_fun(t_vec, X_vec);
% plot(t_vec, des_traj,'--');
error = des_traj - X_vec(3,:);

F_ctrl = controller_fun(t_vec, X_vec);
F_ext = external_fun(t_vec, X_vec);

% Plot the position of the puck, paddle and ceiling
figure;
subplot(3,1,1) 
plot(t_vec,X_vec(1,:),'--');
hold on
plot(t_vec,X_vec(3,:)+p.l0,'--');
plot(t_vec, des_traj+p.l0,'--')
title("Positions");
xlabel('Time (s)')
ylabel('Position (m)')
legend('Actuator Position','Toe Position','Desired Trajectory')

subplot(3,1,2)
plot(t_vec,error,'--');
title("Error");
xlabel('Time (s)')
ylabel('Error (m)')

subplot(3,1,3)
plot(t_vec, F_ctrl);
hold on
plot(t_vec, F_ext);
title("Control Force");
xlabel('Time (s)')
ylabel('Force (N)')
legend('Control Force', 'External Forces/Vibrations')


% Animate the mass
exportVideo = true;
playbackRate = 2;
Animation(p,t_vec,X_vec,des_traj,exportVideo,playbackRate);