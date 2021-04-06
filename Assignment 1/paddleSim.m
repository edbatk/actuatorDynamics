function [t_vec,X_vec,sol_set] = paddleSim(X0,p,paddleFun)
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
assert( isfield(p,'m'))
assert( isfield(p,'e'))
assert( isfield(p,'F'))
assert( isfield(p,'k'))
assert( isfield(p,'c'))
t_start = 0;    % Initial time
t_end = 15;     % Ending time 
dt = 0.01;      % Timestep of the return

% Bind the dynamics function
param_dyn_f = @(t,X)dynamics_f(t,X,p);
param_dyn_c = @(t,X)dynamics_c(t,X,p);
% Bind the event function
event_fun = @(t,X)contactEvent(t,X,p,paddleFun);

% Simulation tolerances
options = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9, ...
    'Events',event_fun);
% Setup data structures
t_vec = t_start:dt:t_end;
X_vec = zeros(length(X0), length(t_vec));
sol_set = {};
contactFlag = false;

% Loop simulation until we reach t_end
while t_start < t_end
    % Run the simulation until t_end or a contact event
    if contactFlag == false
        sol = ode45(param_dyn_f, [t_start,t_end], X0, options);
    else
        sol = ode45(param_dyn_c, [t_start,t_end], X0, options);
    end
    % Concatenate the last ode45 result onto the sol_set cell array. We
    % can't preallocate because we have no idea how many hybrid transitions
    % will occur
    sol_set = [sol_set, {sol}];
    % Setup t_start for the next ode45 call so it is at the end of the 
    % last call 
    t_start = sol.x(end);    
    % Apply the hybrid map, calculate the post impact velocity
    if ~isempty(sol.ie) && sol.ie(end) == 1 % first event ceiling contact
        X0 = ceilingContactMap(sol.xe(end),sol.ye(:,end),p);
    end
    if ~isempty(sol.ie) && sol.ie(end) == 2 % second event paddle contact
        contactFlag = true;
        X0 = paddleContactMap(sol.xe(end),sol.ye(:,end),p,paddleFun);
    else
        contactFlag = false;
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

end % paddleSim

function dX_contact = dynamics_c(t,X,p)
    % t == time
    % X == the state
    % p == parameters structure
    
    x_a  = X(1); % Position actuator
    dx_a = X(2); % Velocity of actuator
    x_t  = X(3); % Inertial mass position
    dx_t = X(4); % Velocity of mass
    x_l  = X(5); % Toe mass
    dx_l = X(6); % Toe Velocity

    % Return the state derivative
    dX_contact = zeros(6,1);
    dX_contact(1) = dx_a;                                  % Velocity actuator
    dX_contact(2) = (p.c*(dx_t - dx_a) + p.k*(x_t - x_a) + p.F)/p.ma; % Acceleration
    dX_contact(3) = dx_t;                                  % Velocity inertial mass
    dX_contact(4) = (p.c*(dx_a - dx_t) + p.k*(x_a - x_t))/(p.mt+p.ml);
    dX_contact(5) = dx_l;
    dX_contact(6) = 0;
end % dynamics

function dX_flight = dynamics_f(t,X,p)
    % t == time
    % X == the state
    % p == parameters structure
    
    x_a  = X(1); % Position actuator
    dx_a = X(2); % Velocity of actuator
    x_t  = X(3); % Inertial load position
    dx_t = X(4); % Velocity of mass
    x_l  = X(5); % Toe mass
    dx_l = X(6); % Toe Velocity

    % Return the state derivative
    dX_flight = zeros(6,1);
    dX_flight(1) = dx_a;                                  % Velocity actuator
    dX_flight(2) = (p.c*(dx_t - dx_a) + p.k*(x_t - x_a) + p.F)/p.ma; % Acceleration
    dX_flight(3) = dx_t;                                  % Velocity inertial mass
    dX_flight(4) = (p.c*(dx_a - dx_t) + p.k*(x_a - x_t))/(p.mt);
    dX_flight(5) = dx_l;
    dX_flight(6) = -9.81;
end % dynamics

function [eventVal,isterminal,direction] = contactEvent(t,X,p,paddleFun)
    % halting event function for ODE simulation. Events are distance to
    % ceiling and distance to paddle
    % Inputs
    % t: time, X: the state, p: parameters structure
    % paddleFun: paddle motion function
    % Outputs
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)
    pos_paddle = paddleFun(t);
    
    dist_wall = X(5) - p.d_wall;
    dist_paddle = pos_paddle - X(5);
    
    eventVal = [dist_wall, dist_paddle];
    isterminal = [1, 1];
    direction = [1, 1];
end % contactEvent

function X_post = ceilingContactMap(t,X_pre,p)
    %The hybrid map to calculate the post impact velocity after ceiling
    %contact
    % t: time
    % X_pre: the state
    % p: parameters structure
    v_post = -p.e*X_pre(6);   
    X_post = [X_pre(5), v_post];
end % ceilingContactMap

function X_post = paddleContactMap(t,X_pre,p,paddleFun)
    %The hybrid map to calculate the post impact velocity after paddle
    %contact
    % t: time
    % X_pre: the state
    % p: parameters structure
    % paddleFun: paddle motion function
    [~, vel_paddle] = paddleFun(t);
    v_post = vel_paddle - p.e*(X_pre(6) - vel_paddle);
    X_post = [X_pre(5), v_post];
end % paddleContactMap
