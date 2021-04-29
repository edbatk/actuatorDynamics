function [t_vec,X_vec,sol_set] = paddleSim2(X0,p,controller_fun)
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
% assert( isfield(p,'m'))
% assert( isfield(p,'e'))
% assert( isfield(p,'F'))
% assert( isfield(p,'k'))
% assert( isfield(p,'c'))
t_start = 0;    % Initial time
t_end = 10;     % Ending time 
dt = 0.01;      % Timestep of the return

% Bind the dynamics function
param_dyn_f = @(t,X)dynamics_f(t,X,p,controller_fun);
param_dyn_c = @(t,X)dynamics_c(t,X,p,controller_fun);
param_dyn_cToe= @(t,X)dynamics_cToe(t,X,p,controller_fun);
% Bind the event function
event_fun = @(t,X)contactEvent(t,X,p,controller_fun);
event_fun2 = @(t,X)contactEvent2(t,X,p,controller_fun);

% Simulation tolerances
options = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9, ...
    'Events',event_fun);
options2 = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9, ...
    'Events',event_fun2);
% Setup data structures
t_vec = t_start:dt:t_end;
X_vec = zeros(length(X0), length(t_vec));
sol_set = {};
contactFlag = false;

% Loop simulation until we reach t_end
while t_start < t_end
    % Run the simulation until t_end or a contact event
%     disp(X0);
    
    if contactFlag == 0
        sol = ode45(param_dyn_f, [t_start,t_end], X0, options);
    else
        sol = ode45(param_dyn_c, [t_start,t_end], X0, options2);
    end
    % Concatenate the last ode45 result onto the sol_set cell array. We
    % can't preallocate because we have no idea how many hybrid transitions
    % will occur
    sol_set = [sol_set, {sol}];
    % Setup t_start for the next ode45 call so it is at the end of the 
    % last call 
    t_start = sol.x(end);    
    % Apply the hybrid map, calculate the post impact velocity
    if ~isempty(sol.ie) && sol.ie(end) == 1% second event load toe contact
        contactFlag = true;
        disp('toe contact')
        X0 = toeContactMap(sol.xe(end),sol.ye(:,end),p);
    elseif ~isempty(sol.ie) && sol.ie(end) == 2% third event load toe break contact
        contactFlag = false;
        disp('break contact')
        X0 = toeBreakContactMap(sol.xe(end),sol.ye(:,end),p);
    elseif ~isempty(sol.ie) && sol.ie(end) == 3 % second event load toe contact
        contactFlag = true;
        disp('min toe')
        X0 = minToeMap(sol.xe(end),sol.ye(:,end),p);
    end
    
%     elseif ~isempty(sol.ie) && sol.ie(end) == 3% 
%         contactFlag = true;
%         toe_min = true;
%         disp('toe min')
%         X0 = minToeMap(sol.xe(end),sol.ye(:,end),p);
%     end
    
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

function dX_contact = dynamics_cToe(t,X,p,controller_fun)
    % t == time
    % X == the state
    % p == parameters structure
    
    x_a  = X(1); % Actuator Position
    dx_a = X(2); % Actuator Velocity
    x_tl  = X(3); % Toe + Load Position
    dx_tl = X(4); % Toe + Load Velocity
    
    F_ctrl = controller_fun(t,X);

    % Return the state derivative
    dX_contact = zeros(6,1);
    dX_contact(1) = dx_a;      
    dX_contact(2) = (p.c*(dx_tl - dx_a) + p.k*(x_tl - x_a) + F_ctrl)/p.ma - 9.81;
    dX_contact(3) = dx_tl;   
    dX_contact(4) = (p.c*(dx_a - dx_tl) + p.k*(x_a - x_tl))/(p.mt+p.ml) -9.81;
    dX_contact(4) = dX_contact(2);
    dX_contact(5) = dX_contact(3);
    dX_contact(6) = dX_contact(4);

end % dynamics

function dX_contact = dynamics_c(t,X,p,controller_fun)
    % t == time
    % X == the state
    % p == parameters structure
    
    x_a  = X(1); % Actuator Position
    dx_a = X(2); % Actuator Velocity
    x_tl  = X(3); % Toe + Load Position
    dx_tl = X(4); % Toe + Load Velocity
    
    F_ctrl = controller_fun(t,X);
    F_ext = 0;
    
    if t > p.inputTimeLower && t < p.inputTimeUpper 
        F_ext = p.appliedForce;
    end

    % Return the state derivative
    dX_contact = zeros(6,1);
    dX_contact(1) = dx_a;
    dX_contact(2) = (p.c*(dx_tl - dx_a) + p.k*(x_tl - x_a) + F_ctrl + F_ext)/p.ma - 9.81; 
    dX_contact(3) = dx_tl;                                  
    dX_contact(4) = (p.c*(dx_a - dx_tl) + p.k*(x_a - x_tl))/(p.mt+p.ml) -9.81;
    dX_contact(5) = dX_contact(3);
    dX_contact(6) = dX_contact(4);

end % dynamics

function dX_flight = dynamics_f(t,X,p,controller_fun)
    % t == time
    % X == the state
    % p == parameters structure 
    
    x_a  = X(1); % Actuator Position 
    dx_a = X(2); % Actuator Velocity
    x_t  = X(3); % Load Position
    dx_t = X(4); % Load Velocity
    x_l  = X(5); % Toe Position
    dx_l = X(6); % Toe Velocity
    
    F_ctrl = controller_fun(t,X);

    % Return the state derivative
    dX_flight = zeros(6,1);
    dX_flight(1) = dx_a; 
    dX_flight(2) = (p.c*(dx_t - dx_a) + p.k*(x_t - x_a) + F_ctrl)/p.ma - 9.81; 
    dX_flight(3) = dx_t;                                  
    dX_flight(4) = (p.c*(dx_a - dx_t) + p.k*(x_a - x_t))/p.mt - 9.81;
    dX_flight(5) = dx_l;
    dX_flight(6) = -9.81;
end % dynamics

function [eventVal,isterminal,direction] = contactEvent(t,X,p,controller_fun)
    % halting event function for ODE simulation. Events are distance to
    % ceiling and distance to paddle
    % Inputs
    % t: time, X: the state, p: parameters structure
    % paddleFun: paddle motion function
    % Outputs
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)    
    
    dist_toe = X(3) - X(5);
    toe_acc = (p.k*(X(1) - X(3)) + p.c*(X(2)-X(4)))/p.mt + .1;
%     block_contact = toe_acc + 9.81; 
    block_contact = 100;
    min_toe = 100;
    
    eventVal = [dist_toe, block_contact, min_toe];
    isterminal = [1, 1, 0];
    direction = [1, -1, 0];
end % contactEvent

function [eventVal,isterminal,direction] = contactEvent2(t,X,p,controller_fun)
    % halting event function for ODE simulation. Events are distance to
    % ceiling and distance to paddle
    % Inputs
    % t: time, X: the state, p: parameters structure
    % paddleFun: paddle motion function
    % Outputs
    % eventVal: Vector of event functions that halt at zero crossings
    % isterminal: if the simulation should halt (yes for both)
    % direction: which direction of crossing should the sim halt (positive)    
    
    
    min_toe = X(3) - X(1) - p.l0*-0.9;
    toe_acc = (p.k*(X(1) - X(3)) + p.c*(X(2)-X(4)))/p.mt + .1;
    block_contact = toe_acc + 9.81; 
    dist_toe = 100;
    
    eventVal = [dist_toe, block_contact, min_toe];
    isterminal = [0, 1, 1];
    direction = [0, -1, 0];
end % contactEvent

function X_post = toeContactMap(t,X_pre,p,controller_fun)
    %The hybrid map to calculate the post impact velocity after paddle
    %contact
    % t: time
    % X_pre: the state
    % p: parameters structure
%     disp('toe contact map activated');
    vtl_post = (p.ml*X_pre(6) + p.mt*X_pre(4))/(p.ml+p.mt);
    X_post = [X_pre(1), X_pre(2), ...
              X_pre(3), vtl_post, ...
              X_pre(3), vtl_post];
end 

function X_post = toeBreakContactMap(t,X_pre,p,controller_fun)
    %The hybrid map to calculate the post impact velocity after paddle
    %contact
    % t: time
    % X_pre: the state
    % p: parameters structure
%     disp('break toe contact map activated');   
    X_post = [X_pre(1), X_pre(2), ...
              X_pre(3), X_pre(4), ...
              X_pre(5), X_pre(6)];
end % paddleContactMap

function X_post = minToeMap(t,X_pre,p,controller_fun)
     dx_a = X_pre(2);
     dx_t = X_pre(4);
%      if X_pre(2)> 0
%          dx_a = 0;
%      end
     if X_pre(4) < 0
         dx_t = 0;
     end
     X_post = [X_pre(1), dx_a, ...               
               X_pre(3) + .05, dx_t, ...
               X_pre(5) + .05, dx_t];
               
%                -0.9*p.l0, 0, ...
%                -0.9*p.l0, 0];
end
