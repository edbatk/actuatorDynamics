% Simulate, plot, and animate a puck bouncing between a wall and a moving
% paddle. This is to illustrate hybrid simulation using event functions
% Author:  Kevin Green 2021

clear;

% Simulation System Constants
p.ma = 1; % mass (kg) of the actuator
p.k = 100; % Stiffness (N/m)
p.c = 1; % Damping (Ns/m)
p.l0 = 2;
p.ml = 0.1;
p.mt = .1;
p.m2 = p.ml + p.mt;
p.e = 0.9;

c.Kp = 50;          % position error feedback gain (N/m)
c.Kd = 5;           % velocity error feedback gain (N/(m/s))
c.amp = 1;
c.freq = 3;
c.sat = 400; % +/- (N) 

X0 = [0, ...        % actuator position (m)
      0];           % wall velocity (m/s)

desDefl = 1;
   
wallFun = @(t) wallMotion(t,c,p);
controller_fun = @(t,X) forceController(t,X,c,p,wallFun);
     
% % Simulate the system
[t_vec,X_vec] = dynamicsSim(X0,p,c,wallFun,controller_fun);
[wall_vec, wall_vel] = wallFun(t_vec);
deflection = wall_vec - X_vec(1,:);

F_ctrl = -controller_fun(t_vec,X_vec);
pos_sat = F_ctrl > c.sat;
neg_sat = F_ctrl < -c.sat;
F_ctrl(pos_sat == 1) = c.sat;
F_ctrl(neg_sat == 1) = -c.sat;

y_des = c.amp*sin(t_vec.*c.freq);
dy_des = c.amp.*c.freq.*cos(t_vec.*c.freq);
% F_spring = p.k*(y_des - p.l0);

desired_force = (p.k*(wall_vec - y_des) + p.c*(wall_vel - dy_des))*p.ma;
spring_force = p.ma*(p.k*(wall_vec - X_vec(1,:)) + p.c*(wall_vel - X_vec(2,:)));
error = desired_force - spring_force;

avg_performance = sum(desired_force ./ spring_force)/length(t_vec);
max_diff = max(abs(spring_force./desired_force));

avg_performances = [];
max_diffs = [];
count = 1;
freqs = [];

for freq = 0.0:0.5:20.0
    c.freq = freq;
    wallFun = @(t) wallMotion(t,c,p);
    controller_fun = @(t,X) forceController(t,X,c,p,wallFun); 
    [t_vec,X_vec] = dynamicsSim(X0,p,c,wallFun,controller_fun);
    [wall_vec, wall_vel] = wallFun(t_vec);
    spring_force = p.ma*(p.k*(wall_vec - X_vec(1,:)) + p.c*(wall_vel - X_vec(2,:)));
    avg_performance = sum(desired_force(500:end) ./ spring_force(500:end))/length(t_vec(500:end));
    max_diff = max(abs(spring_force(1:end)./desired_force(1:end)));
    avg_performances(count) = avg_performance;
    max_diffs(count) = max_diff;
    freqs(count) = freq;
    count = count + 1;
end
% 
figure
subplot(1,1,1)
plot(freqs,avg_performances,'--');
title("Average Frequency Performance Ratios")
xlabel('Frequency (rad/s)')
ylabel('Average Ratio')
legend('Average Ratio')

% subplot(2,1,2)
% plot(freqs,max_diffs,'--');
% title("Maximum Frequency Performance Ratios")
% xlabel('Frequency (rad/s)')
% ylabel('Maximum Ratio')
% legend('Maximum Ratio')

% Plot the position of the puck, paddle and ceiling
figure;
subplot(4,1,1) 
plot(t_vec,X_vec(1,:),'--');
hold on
plot(t_vec,y_des,'--');
title("Positions");
xlabel('Time (s)')
ylabel('Position (m)')
legend('Actuator Position','Desired Position')

subplot(4,1,2) 
plot(t_vec,ones(size(t_vec)) .* desired_force,'--');
hold on
title("System Forces");
plot(t_vec,spring_force,'--');
xlabel('Time (s)')
ylabel('Force (N)')
legend('Desired Force', 'Actual Force')

subplot(4,1,3) 
plot(t_vec,error,'--');
title("Force Error");
xlabel('Time (s)')
ylabel('Error (N)')
legend('Error')

subplot(4,1,4) 
plot(t_vec,F_ctrl,'--');
hold on
plot(t_vec,ones(size(t_vec)) * c.sat,'--');
plot(t_vec,ones(size(t_vec)) * -c.sat,'--');
title("Control Force");
xlabel('Time (s)')
ylabel('Force (N)')
legend('Control Force', 'Max Saturation', 'Miunimum Saturation')

% Animate the mass
exportVideo = true;
playbackRate = 2;
Animation(p,t_vec,X_vec,c,wallFun,exportVideo,playbackRate);