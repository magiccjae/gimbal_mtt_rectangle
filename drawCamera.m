function drawCamera(u,P)

% process inputs to function
NN = 0;
x       = u(1+NN);       % inertial North position
y       = u(2+NN);       % inertial East position
z       = u(3+NN);       % inertial Down position
phi      = u(4+NN);       % roll angle
theta    = u(5+NN);       % pitch angle
psi      = u(6+NN);       % yaw angle
t        = u(7+NN);       % time
NN = NN+7;

target = zeros(P.num_targets,3);
for i=1:P.num_targets
    target(i,:) = [u(1+NN) u(2+NN) u(3+NN)];
    NN = NN+3;
end

% define persistent variables
persistent camera_handle
persistent Vertices
persistent Faces
persistent patchcolors
persistent fov_handle
persistent target_handle

% first time function is called, initialize plot and persistent vars
if t==0,
    figure(1), clf
    [Vertices,Faces,patchcolors] = spacecraftVFC;
    camera_handle = drawSpacecraftBody(Vertices,Faces,patchcolors,x,y,z,phi,theta,psi,[]);
    hold on
    fov_handle = drawFOV(x,y,z,phi,theta,psi,[],P.cam_fov,P.axis_height);
    for i=1:P.num_targets
        target_handle(i,:) = drawTarget(target(i,:), P.target_size, []);
    end
    title('Camera')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    view(32,47)  % set the view angle for figure
    axis([-P.axis_size, P.axis_size, -P.axis_size, P.axis_size, 0, P.axis_height]);
    grid on
    
%     figure(3); hold on; grid on
%     title('targets')
%     for i=1:P.num_targets
%          scatter(target(i,1),target(i,2),200,i,'p');
%     end    
%     xlabel('X')
%     ylabel('Y')
%     set(gca,'YDir','reverse');    
%     axis([-P.axis_size, P.axis_size, -P.axis_size, P.axis_size]);
else
    drawSpacecraftBody(Vertices,Faces,patchcolors,x,y,z,phi,theta,psi,camera_handle);
    drawFOV(x,y,z,phi,theta,psi,fov_handle,P.cam_fov,P.axis_height);
    for i=1:P.num_targets
        drawTarget(target(i,:), P.target_size, target_handle(i,:));
%         figure(3);
%         scatter(target(i,1),target(i,2),[],i,'filled');
    end
end

end

function handle=drawTarget(z, R, handle)
  th = 0:.1:2*pi;
  X = z(1)+ R*cos(th);
  Y = z(2)+ R*sin(th);
  Z = z(3)*ones(length(th));
  
  if isempty(handle),
    handle = fill3(X, Y, Z, 'r');
  else
    set(handle,'XData',X,'YData',Y);
  end
end

function handle = drawFOV(x,y,z,phi,theta,psi,handle,cam_fov,bottom)

pts = [...
    -sin(cam_fov/2),  sin(cam_fov/2)*cos(cam_fov/2),  cos(cam_fov/2)*cos(cam_fov/2);...
    -sin(cam_fov/2), -sin(cam_fov/2)*cos(cam_fov/2),  cos(cam_fov/2)*cos(cam_fov/2);...
     sin(cam_fov/2), -sin(cam_fov/2)*cos(cam_fov/2),  cos(cam_fov/2)*cos(cam_fov/2);...
     sin(cam_fov/2),  sin(cam_fov/2)*cos(cam_fov/2),  cos(cam_fov/2)*cos(cam_fov/2);...
    ]';

pts = rotate(pts,phi,theta,psi);

Vert = [x y z];

% project field of view lines onto ground plane and make correction
% when the projection is above the horizon
for i=1:4,
    % alpha is the angle that the field-of-view line makes with horizon
    alpha = atan2(pts(3,i),norm(pts(1:2,i)));
    if alpha > 0,
        % this is the normal case when the field-of-view line
        % intersects ground plane
        Vert = [...
            Vert;...
            [x+(bottom-z)*pts(1,i)/pts(3,i), y+(bottom-z)*pts(2,i)/pts(3,i), bottom];...
            ];
    else
        % this is when the field-of-view line is above the horizon.  In
        % this case, extend to a finite, but far away (9999) location.
        Vert = [...
            Vert;...
            [x+9999*pts(1,i), y+9999*pts(2,i), bottom];...
            ];
    end
end

Faces = [...
    1, 1, 2, 2;... % x-y face
    1, 1, 3, 3;... % x-y face
    1, 1, 4, 4;... % x-y face
    1, 1, 5, 5;... % x-y face
    2, 3, 4, 5;... % x-y face
    ];

edgecolor      = [1, 1, 1]; % black
footprintcolor = [1, 1, 0];
colors = [edgecolor; edgecolor; edgecolor; edgecolor; footprintcolor];

if isempty(handle),
    handle = patch('Vertices', Vert, 'Faces', Faces,...
        'FaceVertexCData',colors,...
        'FaceColor','flat');
else
    set(handle,'Vertices',Vert,'Faces',Faces);
    drawnow
end

end

function handle = drawSpacecraftBody(V,F,patchcolors,x,y,z,phi,theta,psi,handle)

V = rotate(V, phi, theta, psi);  % rotate vehicle
V = translate(V, x, y, z);  % translate vehicle

if isempty(handle),
    handle = patch('Vertices', V', 'Faces', F,...
        'FaceVertexCData',patchcolors,...
        'FaceColor','flat');
    set(gca,'YDir','reverse');
    set(gca,'ZDir','reverse');    
    
else
    set(handle,'Vertices',V','Faces',F);
    drawnow
end

end


function pts=rotate(pts,phi,theta,psi)

R_roll = [...
    1, 0, 0;...
    0, cos(phi), sin(phi);...
    0, -sin(phi), cos(phi)];
R_pitch = [...
    cos(theta), 0, -sin(theta);...
    0, 1, 0;...
    sin(theta), 0, cos(theta)];
R_yaw = [...
    cos(psi), sin(psi), 0;...
    -sin(psi), cos(psi), 0;...
    0, 0, 1];
R = R_roll*R_pitch*R_yaw;
R = R';     % transposed because we want to rotate points with respect to the camera frame

% rotate vertices
pts = R*pts;

end


function pts = translate(pts,x,y,z)

pts = pts + repmat([x;y;z],1,size(pts,2));

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% defining spacecraft body
function [V,F,patchcolors] = spacecraftVFC

% Define the vertices (physical location of vertices
V = [...
     1    1   0;...   % pt 1
     1   -1   0;...   % pt 2
    -1   -1   0;...   % pt 3
    -1    1   0;...   % pt 4
     1    1  -1;...   % pt 5
     1   -1  -1;...   % pt 6
    -1   -1  -1;...   % pt 7
    -1    1  -1;...   % pt 8
    1.5  1.5  0;...   % pt 9
    1.5 -1.5  0;...   % pt 10
   -1.5 -1.5  0;...   % pt 11
   -1.5  1.5  0;...   % pt 12
    ]';

% define faces as a list of vertices numbered above
F = [...
    1 2 6 5;...  % front
    4 3 7 8;...  % back
    1 5 8 4;...  % right
    2 6 7 3;...  % left
    5 6 7 8;...  % top
    9 10 11 12;...% bottom
    ];

% define colors for each face
myred    = [1, 0, 0];
mygreen  = [0, 1, 0];
myblue   = [0, 0, 1];
myyellow = [1, 1, 0];
mycyan   = [0, 1, 1];

patchcolors = [...
    mygreen;...    % front
    myyellow;...    % back
    myyellow;...     % right
    myyellow;...   % left
    myyellow;...     % top
    myred;...     % bottom
    ];
end
