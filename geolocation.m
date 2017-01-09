function out = geolocation(u,P)

NN = 0;
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

target_location = zeros(2*P.num_targets,1);
NN = 0;
for i=1:P.num_targets
    eps_x = pixel(i,1);
    eps_y = pixel(i,2);
    F = sqrt(P.f^2 + eps_x^2 + eps_y^2);
    ell_c_unit = 1/F * [eps_x; eps_y; P.f];
    ell_i_unit = Rot_v_to_b(camera_phi,camera_theta,camera_psi)' * ell_c_unit;
    k_i_unit = [0; 0; 1];
    denominator = dot(k_i_unit, ell_i_unit);
    h = P.axis_height-camera_z;
    L = h / denominator;
    P_obj_i = [camera_x; camera_y; camera_z] + L * ell_i_unit;
    target_location(1+NN) = P_obj_i(1);
    target_location(2+NN) = P_obj_i(2);
    NN = NN+2;
end

target_distance = zeros(P.num_targets,1);
NN = 0;
for i=1:P.num_targets
    target_distance(i) = norm([target_location(1+NN)-camera_x, target_location(2+NN)-camera_y, P.axis_height-camera_z]);
    NN = NN+2;
end
out = target_distance;

end

function R = Rot_v_to_b(phi,theta,psi)
% Rotation matrix from body coordinates to vehicle coordinates

Rot_v_to_v1 = [...
    cos(psi), sin(psi), 0;...
    -sin(psi), cos(psi), 0;...
    0, 0, 1;...
    ];

Rot_v1_to_v2 = [...
    cos(theta), 0, -sin(theta);...
    0, 1, 0;...
    sin(theta), 0, cos(theta);...
    ];

Rot_v2_to_b = [...
    1, 0, 0;...
    0, cos(phi), sin(phi);...
    0, -sin(phi), cos(phi);...
    ];

R = Rot_v2_to_b * Rot_v1_to_v2 * Rot_v_to_v1;

end
