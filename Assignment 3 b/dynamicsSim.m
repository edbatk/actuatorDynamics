function [t_vec,X_vec,sol_set] = dynamicsSim(X0,p,controller_fun,external_fun)
% Simulates the time response of a spring mass damper
% Given
%   X0:  initial state [position velocity]
%   p.m: mass (kg)
%   p.F: constant force (N)
%   p.e: Coefficient of restitution (0,1]
%   p.d_wall: Distance to the wall (m)
%   p.K: stiffness (N/m)
%   p.C: damping (Ns/m)
%   paddleMotion: function that returns the pos and vel of the paddle
% Returns
%   t_vec: vector of time
%   X_vec: vector of states
% Author: Kevin Green 2021

% Check necessary parameters exist
t_start = 0;    % Initial time
t_end = 5;     % Ending time 
dt = 0.01;      % Timestep of the return

% Bind the dynamics function
param_dyn = @(t,X)dynamics(t,X,p,controller_fun,external_fun);
% Bind the event function

% Simulation tolerances
options = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9);
% Setup data structures
t_vec = t_start:dt:t_end;
X_vec = zeros(length(X0), length(t_vec));
sol_set = {};

% Loop simulation until we reach t_end
while t_start < t_end
    % Run the simulation until t_end or a contact event
%     disp(X0);
    
    sol = ode45(param_dyn, [t_start,t_end], X0, options);

    % Concatenate the last ode45 result onto the sol_set cell array. We
    % can't preallocate because we have no idea how many hybrid transitions
    % will occur
    sol_set = [sol_set, {sol}];
    % Setup t_start for the next ode45 call so it is at the end of the 
    % last call 
    t_start = sol.x(end);    

end % simulation while loop


% Loop to sample the solution structures and build X_vec
for idx = 1:length(sol_set)
    % This sets up a logical vector so we can perform logical indexing
    t_sample_mask = t_vec >= sol_set{idx}.x(1) & t_vec <= sol_set{idx}.x(end);
    % Evaluate the idx solution structure only at the applicable times
    X_eval = deval(sol_set{idx}, t_vec(t_sample_mask));
    % Assign the result to the correct indicies of the return state array
    X_vec(:,t_sample_mask) = X_eval;
end

end % paddleSim

function dX_flight = dynamics(t,X,p,controller_fun,external_fun)
    % t == time
    % X == the state
    % p == parameters structure 
    
    x_a  = X(1); % Actuator Position 
    dx_a = X(2); % Actuator Velocity
    x_t  = X(3); % Load Position
    dx_t = X(4); % Load Velocity
    
    F_ctrl = controller_fun(t,X);
%     F_ext = 0;
%     if t > 3 && t < 3.1
%         F_ext = 10;
%     elseif t > 7 && t < 7.1
%         F_ext = -10;
%     end
    F_ext = external_fun(t,X);

    % Return the state derivative
    dX_flight = zeros(4,1);
    dX_flight(1) = dx_a;                                  
    dX_flight(2) = (p.c*(dx_t - dx_a) + p.k*(x_t - x_a) + F_ctrl + F_ext)/p.ma; 
    dX_flight(3) = dx_t;                                  
    dX_flight(4) = (p.c*(dx_a - dx_t) + p.k*(x_a - x_t) )/p.mt;
end % dynamics