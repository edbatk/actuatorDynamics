function [t_vec,X_vec,sol_set] = paddleSim(X0,p)
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
t_end = 3;     % Ending time 
dt = 0.001;      % Timestep of the return

% Bind the dynamics function
param_dyn_f = @(t,X)dynamics_f(t,X,p);
param_dyn_c = @(t,X)dynamics_c(t,X,p);
% Bind the event function
event_fun = @(t,X)contactEvent(t,X,p);

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
%     disp(X0);
    if contactFlag == 0
        sol = ode45(param_dyn_f, [t_start,t_end], X0, options);
    else
        sol = ode45(param_dyn_c, [t_start,t_end], X0, options);
        disp('contact ode')
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
        disp("ceiling contact")
        X0 = ceilingContactMap(sol.xe(end),sol.ye(:,end),p);
    end
    if ~isempty(sol.ie) && sol.ie(end) == 3 && contactFlag == true % second event load toe break contact
        disp("break contact")
        contactFlag = false;
        X0 = toeBreakContactMap(sol.xe(end),sol.ye(:,end),p);
    end
    if ~isempty(sol.ie) && sol.ie(end) == 2 && contactFlag == false% second event load toe contact
        contactFlag = true;
        disp("in contact")
        X0 = toeContactMap(sol.xe(end),sol.ye(:,end),p); 
    end
    if ~isempty(sol.ie) && sol.ie(end) == 8 % 
        X0 = minActuatorMaxToeMap(sol.xe(end),sol.ye(:,end),p);
    end
    if ~isempty(sol.ie) && sol.ie(end) == 9 % 
        X0 = minActuatorMinToeMap(sol.xe(end),sol.ye(:,end),p);
    end
    if ~isempty(sol.ie) && sol.ie(end) == 10 % 
        X0 = maxActuatorMinToeMap(sol.xe(end),sol.ye(:,end),p);
    end
    if ~isempty(sol.ie) && sol.ie(end) == 11 % 
        X0 = maxActuatorMaxToeMap(sol.xe(end),sol.ye(:,end),p);
    end
    if ~isempty(sol.ie) && sol.ie(end) == 4 % 
        X0 = minActuatorMap(sol.xe(end),sol.ye(:,end),p);
    end
    if ~isempty(sol.ie) && sol.ie(end) == 5 % 
        X0 = maxActuatorMap(sol.xe(end),sol.ye(:,end),p);
    end
    if ~isempty(sol.ie) && sol.ie(end) == 6 % 
        X0 = minToeMap(sol.xe(end),sol.ye(:,end),p);
    end
    if ~isempty(sol.ie) && sol.ie(end) == 7 % 
        X0 = maxToeMap(sol.xe(end),sol.ye(:,end),p);
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
    
    x_a  = X(1); % Actuator Position
    dx_a = X(2); % Actuator Velocity
    x_tl  = X(3); % Toe + Load Position
    dx_tl = X(4); % Toe + Load Velocity    

    % Return the state derivative
    dX_contact = zeros(6,1);
    dX_contact(1) = dx_a;                                  
    dX_contact(2) = (p.c*(dx_tl - dx_a) + p.k*(x_tl - x_a) + p.F)/p.ma;
    dX_contact(3) = dx_tl;                                  
    dX_contact(4) = (p.c*(dx_a - dx_tl) + p.k*(x_a - x_tl))/(p.mt+p.ml) - 9.81;
    dX_contact(5) = dX_contact(3);
    dX_contact(6) = dX_contact(4);

end % dynamics

function dX_flight = dynamics_f(t,X,p)
    % t == time
    % X == the state
    % p == parameters structure 
    
    x_a  = X(1); % Actuator Position 
    dx_a = X(2); % Actuator Velocity
    x_t  = X(3); % Load Position
    dx_t = X(4); % Load Velocity
    x_l  = X(5); % Toe Position
    dx_l = X(6); % Toe Velocity
    
    % actuator toe physical constraints

    % Return the state derivative
    dX_flight = zeros(6,1);
    dX_flight(1) = dx_a;                                  
    dX_flight(2) = (p.c*(dx_t - dx_a) + p.k*(x_t - x_a) + p.F)/p.ma; 
    dX_flight(3) = dx_t;                                  
    dX_flight(4) = (p.c*(dx_a - dx_t) + p.k*(x_a - x_t))/p.mt;
    dX_flight(5) = dx_l;
    dX_flight(6) = -9.81;
end % dynamics

% function [eventVal,isterminal,direction] = contactEvent(t,X,p)
%     % halting event function for ODE simulation. Events are distance to
%     % ceiling and distance to paddle
%     % Inputs
%     % t: time, X: the state, p: parameters structure
%     % paddleFun: paddle motion function
%     % Outputs
%     % eventVal: Vector of event functions that halt at zero crossings
%     % isterminal: if the simulation should halt (yes for both)
%     % direction: which direction of crossing should the sim halt (positive)    
%     dist_wall = X(5) - p.d_wall;
%     dist_toe = X(3) - X(5);      
%     block_contact = (p.c*(X(2) - X(4)) + p.k*(X(1) - X(3))) - 9.81*(p.ml);
%     min_actuator = 1; max_actuator = 1; min_toe = 1; max_toe = 1;
%     min_actuator_max_toe = 1; min_actuator_min_toe = 1;
%     max_actuator_max_toe = 1; max_actuator_min_toe = 1;
%     if X(1) < -1
%         min_actuator = 0;
%     end
%     if X(1) > 1
%         max_actuator = 0;
%     end
%     if X(3)< -0.5*p.l0
%         min_toe = 0;
%     end
%     if X(3) > 2*p.l0
%         max_toe = 0;
%     end
%     if X(1) < -1 && X(3) > 2*p.l0
%         min_actuator_max_toe = 0;
%         min_actuator = 1;
%         max_toe = 1;
%     end
%     if X(1) < -1 && X(3)< -0.5*p.l0
%         min_actuator_min_toe = 0;
%         min_actuator = 1;
%         min_toe = 1;
%     end
%     if X(1) > 1 && X(3)< -0.5*p.l0
%         max_actuator_min_toe = 0;
%         max_actuator = 1;
%         min_toe = 1;
%     end
%     if X(1) > 1 && X(3) > 2*p.l0
%         max_actuator_max_toe = 0;
%         max_actuator = 1;
%         max_toe = 1;
%     end
%     
%     eventVal = [dist_wall, dist_toe, block_contact, min_actuator, max_actuator, min_toe, max_toe, ...
%         min_actuator_max_toe, min_actuator_min_toe, max_actuator_min_toe, max_actuator_max_toe];
%     isterminal = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
%     direction = [1, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
% end % contactEvent

function X_post = ceilingContactMap(t,X_pre,p)
    %The hybrid map to calculate the post impact velocity after ceiling
    %contact
    % t: time
    % X_pre: the state
    % p: parameters structure
    va_post = X_pre(2);
    vt_post = X_pre(4);
    vl_post = -p.e*X_pre(6);
     
    X_post = [X_pre(1), va_post, ...
              X_pre(3), vt_post, ...
              X_pre(5), vl_post];
end % ceilingContactMap

function X_post = toeContactMap(t,X_pre,p)
    %The hybrid map to calculate the post impact velocity after paddle
    %contact
    % t: time
    % X_pre: the state
    % p: parameters structure
    % paddleFun: paddle motion function
%     [~, vel_paddle] = paddleFun(t);
    disp('toe contact map activated');
    va_post = X_pre(2);
    vtl_post = (p.ml*X_pre(6) + p.mt*X_pre(4))/(p.ml+p.mt);
    X_post = [X_pre(1), va_post, ...
              X_pre(3), vtl_post, ...
              X_pre(3), vtl_post];
    disp(X_pre)
    disp(X_post) 
end % paddleContactMap

function X_post = toeBreakContactMap(t,X_pre,p)
    %The hybrid map to calculate the post impact velocity after paddle
    %contact
    % t: time
    % X_pre: the state
    % p: parameters structure
%     disp('break toe contact map activated');
    va_post = X_pre(2);
    vt_post = (p.ml*p.mt+p.mt^2)*X_pre(4)/((p.mt^2 - p.mt*p.ml)^.5);
    vl_post = (p.ml + p.mt)*X_pre(4)/p.ml - (p.mt/p.ml)*((p.ml*p.mt + p.mt^2)*X_pre(4)/((p.mt^2 - p.mt*p.ml)^.5));
    X_post = [X_pre(1), va_post, ...
              X_pre(3), vt_post, ...
              X_pre(5), vl_post];
    disp(X_pre)
    disp(X_post)
end % paddleContactMap

function X_post = minActuatorContactMap(t,X_pre,p)
     disp('min actuator limit')
     X_post = [-1, 0, ...
              X_pre(3), X_pre(4), ...
              X_pre(5), X_pre(6)];
end

function X_post = maxActuatorContactMap(t,X_pre,p)
     disp('max actuator limit')
     X_post = [1, 0, ...
              X_pre(3), X_pre(4), ...
              X_pre(5), X_pre(6)];
end

function X_post = minToeMap(t,X_pre,p)
     disp('min toe limit')
     X_post = [X_pre(1), X_pre(2), ...
              -0.5*p.l0, 0, ...
              X_pre(5), X_pre(6)];
end

function X_post = maxToeMap(t,X_pre,p)
     disp('max toe limit')
     X_post = [X_pre(1), X_pre(2), ...
              2*p.l0, 0, ...
              X_pre(5), X_pre(6)];
end

function X_post = minActuatorMaxToeMap(t,X_pre,p)
     disp('max toe limit')
     X_post = [-1, 0, ...
              2*p.l0, 0, ...
              X_pre(5), X_pre(6)];
end

function X_post = minActuatorMinToeMap(t,X_pre,p)
     disp('max toe limit')
     X_post = [-1, 0, ...
              -0.5*p.l0, 0, ...
              X_pre(5), X_pre(6)];
end

function X_post = maxActuatorMaxToeMap(t,X_pre,p)
     disp('max toe limit')
     X_post = [1, 0, ...
              2*p.l0, 0, ...
              X_pre(5), X_pre(6)];
end

function X_post = maxActuatorMinToeMap(t,X_pre,p)
     disp('max toe limit')
     X_post = [1, 0, ...
              -0.5*p.l0, 0, ...
              X_pre(5), X_pre(6)];
end
