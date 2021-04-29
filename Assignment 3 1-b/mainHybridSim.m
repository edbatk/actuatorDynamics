% Simulate, plot, and animate a puck bouncing between a wall and a moving
% paddle. This is to illustrate hybrid simulation using event functions
% Author:  Kevin Green 2021

clear;

X0 = [ 0, ...       % actuator position (m)
       0, ...       % actuator velocity (m/s)
       0, ...       % toe position (m) DATUMMM
       0, ...       % toe velocity (m/s)
       0.01, ...       % load position (m)
       0];          % load velocity (m/s)

% Simulation System Constants
p.ma = 3; % mass (kg) of the actuator
p.k = 100; % Stiffness (N/m)
p.c = 3; % Damping (Ns/m)
p.l0 = 2;
p.ml = 0.1;
p.mt = .1;
p.m2 = p.ml + p.mt;
p.appliedForce = 100;
p.inputTimeLower = 5;
p.inputTimeUpper = 5.1;

c.Kp = 150;          % position error feedback gain (N/m)
c.Kd = 15;           % velocity error feedback gain (N/(m/s))
c.amp = 1;
c.freq = 3;

controller_fun = @(t,X) forceController(p,t,X,c); 
     
% % Simulate the system
[t_vec,X_vec] = paddleSim2(X0,p,controller_fun);
F_ctrl = controller_fun(t_vec,X_vec);

extForceArray = t_vec;
extForceArray(t_vec < p.inputTimeLower) = 0;
extForceArray(t_vec >= p.inputTimeLower) = p.appliedForce;
extForceArray(t_vec > p.inputTimeUpper) = 0;

% Plot the position of the puck, paddle and ceiling
figure;
subplot(2,1,1) 
plot(t_vec,X_vec(1,:),'--');
hold on
plot(t_vec,X_vec(3,:)+p.l0,'--');
plot(t_vec,X_vec(5,:)+p.l0,'--');
title("Positions");
xlabel('Time (s)')
ylabel('Position (m)')
legend('Actuator Position','Toe Position','Load Position')

% Plot the System Forces
subplot(2,1,2)
plot(t_vec,F_ctrl,'--');
hold on
plot(t_vec,extForceArray,'--');
title("Applied Force")
xlabel('Time (s)')
ylabel('Force (N)')
legend('Disturbance Force')

% Animate the mass
exportVideo = true;
playbackRate = 2;
paddleAnimation(p,t_vec,X_vec,exportVideo,playbackRate);