function output = velocity_command(u, P)

NN = 0;
target_depth = zeros(P.num_targets,1);
for i=1:P.num_targets
    target_depth(i) = u(1+NN);
    NN = NN+1;
end

camera_x = u(1+NN);     % camera x location
camera_y = u(2+NN);     % camera y location
camera_z = u(3+NN);     % camera z location
camera_phi = u(4+NN);   % camera phi
camera_theta = u(5+NN); % camera theta
camera_psi = u(6+NN);   % camera psi
NN = NN+6;

pixel = zeros(P.num_targets,3);
for i=1:P.num_targets
    pixel(i,:) = [u(1+NN) u(2+NN) u(3+NN)];
    NN = NN+3;
end

% importance vector
importance = [100 10 10 10 10 10];
% normalize
importance = importance / sum(importance);

% only consider feature points in camera FOV
pixel1 = [];
for i=1:P.num_targets
    if pixel(i,3)~=0
        pixel1 = [pixel1; pixel(i,:)];
    end
end

% Get the convex hull of target feature points in image plane. This convex hull
% is used to create a rectangle that includes all the feature points. The
% four vertices of rectangle is used to get the mean and variance to the
% controller.

% K = convhull(pixel(:,1), pixel(:,2));
% figure(2); hold on
% persistent plot_handle
% if isempty(plot_handle)
%     plot_handle = plot(pixel(K,1),pixel(K,2),'b');
% else
%     delete(plot_handle)
%     plot_handle = plot(pixel(K,1),pixel(K,2),'b');
% end

% when the most important target is a point that makes the outer rectangle,
% it might go out of FOV soon. So we want to re-center the camera view by
% shifting the center closer to the important target.
[min_x, i1] = min(pixel1(:,1));
[max_x, i2] = max(pixel1(:,1));
[min_y, i3] = min(pixel1(:,2));
[max_y, i4] = max(pixel1(:,2));
[most_important, important_i] = max(importance);

[i1 i2 i3 i4]

% creating 4 vertices of the rectangle that includes all the target feature
% points
vertices = [min_x, min_y;...
            max_x, min_y;...
            max_x, max_y;...
            min_x, max_y;...
            min_x, min_y];
        
% plotting the outer rectangle
figure(2); hold on
persistent plot_handle
if isempty(plot_handle)
    plot_handle = plot(vertices(:,1),vertices(:,2),'b');
else
    delete(plot_handle)
    plot_handle = plot(vertices(:,1),vertices(:,2),'b');
end
vertices = vertices(1:4,:);

% ==========multiple targets==========
% constructing the Image Jacobian matrix
num_vertex = 4;
L = zeros(num_vertex*2,6);
depth = mean(target_depth);
NN = 0;
for i=1:num_vertex
    L(1+NN,:) = [-1/depth      0       vertices(i,1)/depth      vertices(i,1)*vertices(i,2)    -(1+vertices(i,1)^2)    vertices(i,2)];
    L(2+NN,:) = [   0      -1/depth    vertices(i,2)/depth      1+vertices(i,2)^2      -vertices(i,1)*vertices(i,2)   -vertices(i,1)];
    NN = NN+2;
end

% task function Jacobian for mean and other terms in eq(11)
mean_x = mean(vertices(:,1));
mean_y = mean(vertices(:,2));

mean_x1 = -999;
% shift mean if the most important target makes the outer rectangle
% calculate the vector pointing to each feature points
if i1==important_i | i2==important_i | i3==important_i | i4==important_i
    [num_pixel, dummy] = size(pixel1);
    vector_to_points = zeros(num_pixel,2);
    shifting = zeros(num_pixel,2);    
    for i=1:num_pixel
        vector_to_points(i,:) = [pixel1(i,1)-mean_x, pixel1(i,2)-mean_y];
        shifting(i,:) = importance(i)*vector_to_points(i,:);
    end
    shift_x = sum(shifting(:,1));
    shift_y = sum(shifting(:,2));
    mean_x1 = mean_x + shift_x;
    mean_y1 = mean_y + shift_y;
end

% draw the center of rectangle
figure(2); hold on
persistent center_handle
persistent shifted_handle
if isempty(center_handle)
    center_handle = scatter(mean_x,mean_y,100,'k','p','filled');
else
    delete(center_handle)
    center_handle = scatter(mean_x,mean_y,100,'k','p','filled');
    if mean_x1 ~= -999
        if isempty(shifted_handle)
            shifted_handle = scatter(mean_x1,mean_y1,100,'k','*');
        else
            delete(shifted_handle)
            shifted_handle = scatter(mean_x1,mean_y1,100,'k','*');            
        end
    end
end

tf_m_error = [mean_x; mean_y];  % mean task function error. m_desired = [0; 0]

m_dot_m = -P.J_m_pinv*(P.gamma_m*tf_m_error - P.tf_m_dot_desired + P.c_epsillon^2/(P.k*P.kappa_m)*tf_m_error);

if P.num_targets==1
    V_c = pinv(L)*m_dot_m;
elseif P.num_targets > 1
    % task function Jacobian for variance and other terms in eq(14)
    variance_x = var(vertices(:,1));
    variance_y = var(vertices(:,2));
    temp_jv = zeros(2,2*P.num_vertex);
    NN = 0;
    for i=1:P.num_vertex
        temp_jv(1,1+NN) = vertices(i,1)-mean_x;
        temp_jv(2,2+NN) = vertices(i,2)-mean_y;
        NN = NN+2;
    end
    J_v = 2/P.k*temp_jv;
    J_v_pinv = J_v'*inv(J_v*J_v');  % right psedoinverse of J_v
    tf_v_error = [variance_x; variance_y] - P.tf_v_desired;
    m_dot_v = -J_v_pinv*(P.gamma_v*tf_v_error - P.tf_v_dot_desired + P.c_v^2*P.c_epsillon^2/P.kappa_v*tf_v_error);
    
    V_c = pinv(L)*(m_dot_m + m_dot_v);
%     V_c = pinv(L)*(m_dot_m);
end

V_c = sat(V_c);
output = V_c;

% output = [0; 0; 0; 0; 0; 0];
end

function out = sat(u)
    v_x = u(1);
    v_y = u(2);
    v_z = u(3);
    w_x = u(4);
    w_y = u(5);
    w_z = u(6);
    
    if v_x > 20
        v_x = 20;
    elseif v_x < -20
        v_x = -20;
    end
    
    if v_y > 20
        v_y = 20;
    elseif v_y < -20
        v_y = -20;
    end

    if v_z > 20
        v_z = 20;
    elseif v_z < -20
        v_z = -20;
    end

    if w_x > 1
        w_x = 1;
    elseif w_x < -1
        w_x = -1;
    end

    if w_y > 1
        w_y = 1;
    elseif w_y < -1
        w_y = -1;
    end

    if w_z > 1
        w_z = 1;
    elseif w_z < -1
        w_z = -1;
    end
    
    out = [v_x; v_y; v_z; w_x; w_y; w_z];
end
