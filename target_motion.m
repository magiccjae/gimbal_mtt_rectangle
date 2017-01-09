function [sys,x0,str,ts] = target_motion(t,x,u,flag,P)
% define motion of target vehicle

switch flag,
    
    %%%%%%%%%%%%%%%%%%
    % Initialization %
    %%%%%%%%%%%%%%%%%%
    case 0,
        [sys,x0,str,ts]=mdlInitializeSizes(P);
        
        %%%%%%%%%%%%%%%
        % Derivatives %
        %%%%%%%%%%%%%%%
    case 1,
        sys=mdlDerivatives(t,x,u,P);
        
        %%%%%%%%%%
        % Update %
        %%%%%%%%%%
    case 2,
        sys=mdlUpdate(t,x,u,P);
        
        %%%%%%%%%%%
        % Outputs %
        %%%%%%%%%%%
    case 3,
        sys=mdlOutputs(t,x,P);
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % GetTimeOfNextVarHit %
        %%%%%%%%%%%%%%%%%%%%%%%
    case 4,
        sys=mdlGetTimeOfNextVarHit(t,x,u);
        
        %%%%%%%%%%%%%
        % Terminate %
        %%%%%%%%%%%%%
    case 9,
        sys=mdlTerminate(t,x,u);
        
        %%%%%%%%%%%%%%%%%%%%
        % Unexpected flags %
        %%%%%%%%%%%%%%%%%%%%
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
        
end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = P.num_targets*3; % position
sizes.NumDiscStates  = P.num_targets*3; % velocity
sizes.NumOutputs     = P.num_targets*3; % output position
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1; % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%

% initial position of target(target can move any direction from (0,0))
z0 = zeros(P.num_targets*3,1);
v0 = zeros(P.num_targets*3,1);
NN = 0;
for i=1:P.num_targets
    z0(1+NN) = P.target_initial*rand-P.target_initial/2;
    z0(2+NN) = P.target_initial*rand-P.target_initial/2;
    z0(3+NN) = P.axis_height;
    v0(1+NN) = P.target_velocity*rand-P.target_velocity/2;
    v0(2+NN) = P.target_velocity*rand-P.target_velocity/2;
    v0(3+NN) = 0;
    NN = NN+3;
end

x0 = zeros(P.num_targets*6,1);
NN = 0;
% continuous state
for i=1:P.num_targets
    x0(1+NN) = z0(1+NN);
    x0(2+NN) = z0(2+NN);
    x0(3+NN) = z0(3+NN);
    NN = NN+3;
end

NNN = 0;
% discrete velocity
for i=1:P.num_targets
    x0(1+NN) = v0(1+NNN);
    x0(2+NN) = v0(2+NNN);
    x0(3+NN) = v0(3+NNN);
    NN = NN+3;
    NNN = NNN+3;
end

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,P)

vel = zeros(P.num_targets*3,1);
NN = 0;
NNN = P.num_targets*3;
for i=1:P.num_targets
    vel(1+NN) = x(1+NNN);
    vel(2+NN) = x(2+NNN);
    vel(3+NN) = x(3+NNN);
    NN = NN+3;
    NNN = NNN+3;
end

sys = vel;

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u,P)

z = zeros(P.num_targets*3,1);
vel = zeros(P.num_targets*3,1);
NN = 0;
% continuous states
for i=1:P.num_targets
    z(1+NN) = x(1+NN);
    z(2+NN) = x(2+NN);
    z(3+NN) = x(3+NN);
    NN = NN+3;
end
NNN = 0;
% discrete states
for i=1:P.num_targets
    vel(1+NNN) = x(1+NN);
    vel(2+NNN) = x(2+NN);
    vel(3+NNN) = x(3+NN);
    NN = NN+3;
    NNN = NNN+3;
end

NN = 0;
for i=1:P.num_targets
    target_turning_angle = 20;  % how much the target can turn in one time step
    if mod(t,1)==0      % every second target can change heading direction
        condition = 1;
        while condition
            temp_vel_x = P.target_velocity*rand-P.target_velocity/2;
            temp_vel_y = P.target_velocity*rand-P.target_velocity/2;
            next_vector = [temp_vel_x; temp_vel_y];
            previous_vector = [vel(1+NN); vel(2+NN)];
            angle = acos(dot(next_vector, previous_vector)/(norm(next_vector)*norm(previous_vector)));
            if angle*180/pi <= target_turning_angle
                condition = 0;
                vel(1+NN) = temp_vel_x;
                vel(2+NN) = temp_vel_y;
                vel(3+NN) = 0;
            end
        end
    end
    NN = NN+3;
end
sys = vel;

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,P)

NN = 0;
z = zeros(P.num_targets*3,1);
% continuous states
for i=1:P.num_targets
    z(1+NN) = x(1+NN);
    z(2+NN) = x(2+NN);
    z(3+NN) = x(3+NN);
    NN = NN+3;
end

sys = z;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end