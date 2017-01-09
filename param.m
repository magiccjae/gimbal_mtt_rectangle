clear
close all
clc

P.num_targets = 6;

% drawCamera parameters
P.axis_size = 200;
P.axis_height = 200;
P.cam_fov = 45*(pi/180);
P.target_size = 1;

% camera_motion parameters
P.initial_camera_z = 50;

% target_motion parameters
P.target_initial = 100;
P.target_velocity = 10;

% camera parameters
P.cam_pix = 800;
P.pixelnoise = 0;       % (pixels) - variance of the pixel noise
P.f = (P.cam_pix/2)/tan(P.cam_fov/2);   % focal range in pixel

% velocity command parameters
P.num_vertex = 4;
P.k = P.num_vertex;  % the number of targets
temp_eyes = zeros(2*P.num_vertex,2);
NN = 0;
for i=1:P.num_vertex
    temp_eyes(1+NN,1) = 1;
    temp_eyes(2+NN,2) = 1;
    NN = NN+2;
end
P.J_m = 1/P.k * temp_eyes';
P.J_m_pinv = P.J_m'*inv(P.J_m*P.J_m');      % right psedoinverse of J_m
P.gamma_m = 10;    % <========== constant gain, tuning parameter
P.tf_m_dot_desired = 1;   % desired task function derivative
P.c_epsillon = 10;   % see eq(4)
P.kappa_m = 1000;    % <========== constant gain, tuning parameter
P.gamma_v = 1;    % <========== constant gain, tuning parameter
P.tf_v_desired = [20000; 20000];
P.tf_v_dot_desired = 10;
P.c_v = 1;      % constant bound
P.kappa_v = 1000000;   % <========== constant gain, tuning parameter


