
function [t_vec,X_vec,sol_set] = walkingSim(X0,p,z,td_control)
clear global step1
setGlobalStep(0);

t_start = 0;    % Initial time
t_end = 5;     % Ending time 
dt = 0.01;      % Timestep of the return

% Bind the dynamics function
param_dyn_ss = @(t,X)dynamics_ss(t,X,p,z,td_control);
param_dyn_ds = @(t,X)dynamics_ds(t,X,p,z,td_control);
% Bind the event function
event_fun_ss = @(t,X)contactEvent_ss(t,X,p,z,td_control);
event_fun_ds = @(t,X)contactEvent_ds(t,X,p,z,td_control);

% Simulation tolerances
options_ss = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9, ...
    'Events',event_fun_ss);
options_ds = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9, ...
    'Events',event_fun_ds);
% Setup data structures
t_vec = t_start:dt:t_end;
X_vec = zeros(length(X0), length(t_vec));
sol_set = {};
single_support = true;

% Loop simulation until we reach t_end
while t_start < t_end 
    if single_support == 1
        disp('single stance')
        sol = ode45(param_dyn_ss, [t_start,t_end], X0, options_ss);
    else
        disp('double stance')
        sol = ode45(param_dyn_ds, [t_start,t_end], X0, options_ds);
    end
    % Concatenate the last ode45 result onto the sol_set cell array. We
    % can't preallocate because we have no idea how many hybrid transitions
    % will occur
    sol_set = [sol_set, {sol}];
    % Setup t_start for the next ode45 call so it is at the end of the 
    % last call 
    t_start = sol.x(end);    
    % Apply the hybrid map, calculate the post impact velocity
    if ~isempty(sol.ie) && sol.ie(end) == 1% single support event
        disp('entering double stance')
        single_support = 0;
        X0 = dsMap(sol.xe(end),sol.ye(:,end),p);
    elseif ~isempty(sol.ie) && (sol.ie(end) == 2 || sol.ie(end) == 3) % double support event
        disp('entering single stance')
        single_support = true;      
        setGlobalStep(getGlobalStep + p.step)
        setGlobalTimes(t_start)
        X0 = ssMap(sol.xe(end),sol.ye(:,end),p);
        disp(getGlobalStep)
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

function dX = dynamics_ss(t,X,p,z,td_control)  
    x_x  = X(1); % X Position
    dx_x = X(2); % X Velocity
    x_y  = X(3); % Y Position
    dx_y = X(4); % Y Velocity
    
    x_t1 = getGlobalStep;
%     disp(x_t1)
    
%     l1 = sqrt((x_x - p.x_t1).^2 + (x_y - p.y_t1).^2);
    l1 = sqrt((x_x - x_t1).^2 + (x_y - p.y_t1).^2);
    F1 = p.k*(p.l0 - l1);
    
    vel = sqrt(dx_x.^2 + dx_y.^2);
    F1 = p.k*(p.l0 - l1) + p.c*(0 - vel);
    
    if F1 < 0
        F1 = 0;
    end

    % Return the state derivative
    dX = zeros(4,1);
    dX(1) = dx_x;                                  
%     dX(2) = F1*(x_x - p.x_t1)/(p.mw*l1);
    dX(2) = F1*(x_x - x_t1)/(p.mw*l1);
    dX(3) = dx_y;   
    dX(4) = F1*(x_y - p.y_t1)/(p.mw*l1) -9.81;
%     disp('single stance')
%     disp(getGlobalStep)
%     val_print = fprintf('l1: %4.2f', l1);
%     disp(val_print);
end % dynamics

function dX = dynamics_ds(t,X,p,z,td_control)  
    x_x  = X(1); % X Position
    dx_x = X(2); % X Velocity
    x_y  = X(3); % Y Position
    dx_y = X(4); % Y Velocity
    
    x_t1 = getGlobalStep;
    x_t2 = x_t1 + p.step;
    
%     l1 = sqrt((x_x - p.x_t1).^2 + (x_y - p.y_t1).^2);
    l1 = sqrt((x_x - x_t1).^2 + (x_y - p.y_t1).^2);
    F1 = p.k*(p.l0 - l1);
    
    vel = sqrt(dx_x.^2 + dx_y.^2);
    F1 = p.k*(p.l0 - l1) + p.c*(0 - vel);
    
    if F1 < 0
        F1 = 0;
    end
%     l2 = sqrt((x_x - p.x_t2).^2 + (x_y - p.y_t2).^2);
    l2 = sqrt((x_x - x_t2).^2 + (x_y - p.y_t2).^2);
    F2 = p.k*(p.l0 - l2);
    
    F2 = p.k*(p.l0 - l2) + p.c*(0 - vel);
    
    if F2 < 0
        F2 = 0;
    end

    % Return the state derivative
    dX = zeros(4,1);
    dX(1) = dx_x;                                  
%     dX(2) = F1*(x_x - p.x_t1)/(p.mw*l1) + F2*(x_x - p.x_t2)/(p.mw*l2);
    dX(2) = F1*(x_x - x_t1)/(p.mw*l1) + F2*(x_x - x_t2)/(p.mw*l2);
    dX(3) = dx_y;   
    dX(4) = F1*(x_y - p.y_t1)/(p.mw*l1) + F2*(x_y - p.y_t2)/(p.mw*l2) -9.81;
%     disp('double stance')
%     disp(getGlobalStep)
%     val_print = fprintf('l1: %4.2f, l2: %4.2f',l1,l2);
%     disp(val_print);
    
end % dynamics

% function [eventVal,isterminal,direction] = contactEvent_ss(t,X,p,z,td_control)  
%     if X(4) < 0
%         negComYVel = 0;
%     else
%         negComYVel = 100;
%     end
% %     negComYVel = X(4);
%     tdAngle =  X(3) - p.l0*sin(p.tdAngle) ;
%     ss2ds = negComYVel + tdAngle;
%     x_t1 = getGlobalStep;
% %     l1 = sqrt((X(1) - p.x_t1).^2 + (X(3) - p.y_t1).^2);
%     l1 = sqrt((X(1) - x_t1).^2 + (X(3) - p.y_t1).^2);
%     ds2ss = p.l0 - l1;
% %     ds2ss = 1;
%     
%     eventVal = [ss2ds, ds2ss];
%     isterminal = [1, 1];
%     direction  = [1, -1];
% end % contactEvent

function [eventVal,isterminal,direction] = contactEvent_ss(t,X,p,z,td_control)  
    if X(4) < 0
        negComYVel = 0;
    else
        negComYVel = 100;
    end
%     negComYVel = X(4);
    tdAngle =  X(3) - p.l0*sin(p.tdAngle) ;
%     disp(tdAngle);
    ss2ds = negComYVel + tdAngle;
%     disp(ss2ds);
    ds2ss = 100;
    back_up = ds2ss;
    eventVal = [ss2ds, ds2ss, back_up];
    isterminal = [1, 0, 0];
    direction  = [-1, 0, 0];
end % contactEvent

function [eventVal,isterminal,direction] = contactEvent_ds(t,X,p,z,td_control)  
    x_t1 = getGlobalStep;
    l1 = sqrt((X(1) - x_t1).^2 + (X(3) - p.y_t1).^2);
    x_t2 = getGlobalStep + 0.5;
    l2 = sqrt((X(1) - x_t2).^2 + (X(3) - p.y_t2).^2);
    
    ds2ss = p.l0 - l1;
    
    back_up = p.l0 + 0.11;
    if back_up >= 1
        back_up = 0.1;
    end
    back_up = back_up - back_up;
    back_up = 10;
    
%     disp(ds2ss)
    val_print = sprintf('X(1): %4.2f, x_tl: %4.2f, l1: %4.2f',X(1),x_t1,l1);
%     disp(val_print)
    ss2ds = 100;
    
    eventVal = [ss2ds, ds2ss, back_up];
    isterminal = [0, 1, 1];
    direction  = [0, 0, 0];
end % contactEvent

function X_post = ssMap(t,X_pre,p,z,td_control)
    X_post = [X_pre(1), X_pre(2), ...
              X_pre(3), X_pre(4)];
end 

function X_post = dsMap(t,X_pre,p,z,td_control)  
    X_post = [X_pre(1), X_pre(2), ...
              X_pre(3), X_pre(4)];
end 



