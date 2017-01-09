function [sys,x0,str,ts] = camera_motion(t,x,u,flag,P)
% define motion of floating camera

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
        sys=mdlDerivatives(t,x,u);
        
        %%%%%%%%%%
        % Update %
        %%%%%%%%%%
    case 2,
        sys=mdlUpdate(t,x,u,P);
        
        %%%%%%%%%%%
        % Outputs %
        %%%%%%%%%%%
    case 3,
        sys=mdlOutputs(t,x);
        
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

sizes.NumContStates  = 12; % position, velocity
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;  % position
sizes.NumInputs      = 6;  % command velocity
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%

% initial position and velocity of the camera
position0 = [0; 0; P.initial_camera_z; 0; 0; 0];
velocity0=[0; 0; 0; 0; 0; 0];

% continuous states
NN = 0;
x0(1+NN) = position0(1);
x0(2+NN) = position0(2);
x0(3+NN) = position0(3);
x0(4+NN) = position0(4);
x0(5+NN) = position0(5);
x0(6+NN) = position0(6);
x0(7+NN) = velocity0(1);
x0(8+NN) = velocity0(2);
x0(9+NN) = velocity0(3);
x0(10+NN) = velocity0(4);
x0(11+NN) = velocity0(5);
x0(12+NN) = velocity0(6);

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
function sys=mdlDerivatives(t,x,u)
x_dot = u(1);
y_dot = u(2);
z_dot = u(3);
phi_dot = u(4);
theta_dot = u(5);
psi_dot = u(6);
x_ddot = 0;
y_ddot = 0;
z_ddot = 0;
phi_ddot = 0;
theta_ddot = 0;
psi_ddot = 0;

camera_z = x(3);
if camera_z <= 0
    z_dot = 0;
end

sys = [x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot;...
       x_ddot; y_ddot; z_ddot; phi_ddot; theta_ddot; psi_ddot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u,P)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x)
camera_x     = x(1);
camera_y     = x(2);
camera_z     = x(3);
camera_phi   = x(4);
camera_theta = x(5);
camera_psi   = x(6);

sys = [camera_x; camera_y; camera_z; camera_phi; camera_theta; camera_psi];

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