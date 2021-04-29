function [t_vec,X_vec,sol_set] = dynamicsSim(X0,p,c,wallFun,controller_fun)
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

t_start = 0;    % Initial time
t_end = 10;     % Ending time 
dt = 0.01;      % Timestep of the return

% Bind the dynamics function
param_dyn = @(t,X)dynamics(t,X,p,c,wallFun,controller_fun);
event_fun = @(t,X)contactEvent(t,X,p,wallFun,controller_fun);

% Simulation tolerances
options = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9 , ...
    'Events',event_fun);
% Setup data structures
t_vec = t_start:dt:t_end;
X_vec = zeros(length(X0), length(t_vec));
sol_set = {};

% Loop simulation until we reach t_end
while t_start < t_end
    % Run the simulation until t_end or a contact event
    sol = ode45(param_dyn, [t_start,t_end], X0, options);
    % Concatenate the last ode45 result onto the sol_set cell array. We
    % can't preallocate because we have no idea how many hybrid transitions
    % will occur
    sol_set = [sol_set, {sol}];
    % Setup t_start for the next ode45 call so it is at the end of the 
    % last call 
    t_start = sol.x(end);
    if ~isempty(sol.ie) && sol.ie(end) == 1% second event load toe contact
        X0 = contactMap(sol.xe(end),sol.ye(:,end),p, wallFun);
    end
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

end % sim

function dX = dynamics(t,X,p,c,wallFun,controller_fun)
    % t == time
    % X == the state
    % p == parameters structure 
    
    x_a  = X(1); % Actuator Position 
    dx_a = X(2); % Actuator Velocity
    [p_w, v_w, a_w] = wallFun(t);
    
    F_ctrl = controller_fun(t,X);
    
    if F_ctrl > c.sat
        F_ctrl = c.sat;
    elseif F_ctrl < -c.sat
        F_ctrl = -c.sat;
    end

    % Return the state derivative
    dX = zeros(2,1);
    dX(1) = dx_a;                                  
    dX(2) = (p.k*(p_w - x_a) + p.c*(v_w - dx_a) + F_ctrl)/p.ma; 
end % dynamics

function [eventVal,isterminal,direction] = contactEvent(t,X,p,wallFun,controller_fun)
    % halting event function for ODE simulation. Events are distance to
    % ceiling and distance to paddle
    % Inputs
    % t: time, X: the state, p: parameters structure
    % paddleFun: paddle motion function
    % Outputs
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)    
    
    [p_w, ~] = wallFun(t);
    dist = p_w - X(1);
    
    eventVal = [dist];
    isterminal = [1];
    direction = [-1];
end % contactEvent

function X_post = contactMap(t,X_pre,p,wallFun)
    %The hybrid map to calculate the post impact velocity after paddle
    %contact
    % t: time
    % X_pre: the state
    % p: parameters structure
    % paddleFun: paddle motion function
    [~, v_w] = wallFun(t);
    v_post = v_w - p.e*(X_pre(2) - v_w);
    X_post = [X_pre(1), v_post];
end % paddleContactMap

