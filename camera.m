% camera
%
% simulates camera
% input is
%    uu(1:3) - target position
%    uu(4:6) - target velocity
%
% output is
%    px = x-pixel
%    py = y-pixel
%    ps = size of blob in image plane (in pixels)
%
function out = camera(uu,P)

% process inputs
NN = 0;
x    = uu(1+NN); % camera X position
y    = uu(2+NN); % camera Y position
z    = uu(3+NN); % Camera Z position
phi   = uu(4+NN); % roll angle
theta = uu(5+NN); % pitch angle
psi   = uu(6+NN); % yaw angle
NN = NN+6;

t     = uu(1+NN); % time
NN = NN+1;

target = zeros(P.num_targets,3);
for i=1:P.num_targets
    target(i,:) = [uu(1+NN) uu(2+NN) uu(3+NN)];
    NN = NN+3;
end

% persistent variable
persistent blob_handle  % figure handle for blob

% determine pixel location and size
pixel_location = zeros(P.num_targets,3);
for i=1:P.num_targets
    pixel_location(i,:) = cameraProjection(target(i,1),target(i,2),target(i,3),x,y,z,phi,theta,psi,P);
end

% pixel_location = [];
% for i=1:P.num_targets
%     if temp_pixel_location(i,3)~=0
%         pixel_location = [pixel_location; temp_pixel_location(i,:)];
%     end    
% end
% 
% [num_pixel, dummy] = size(pixel_location);
% 
% num_pixel = num_pixel
% pixel_location

if t==0,  % initialize the plot
    figure(2), clf, hold on
    title('Camera View')    
    % plot camera view
    h=subplot('position',[0.1, 0.1, 0.8, 0.8]);
    tmp = P.cam_pix/2;
    set(h,'XAxisLocation','top','XLim',[-tmp,tmp],'YLim',[-tmp,tmp]);
    axis ij
    hold on
    for i=1:P.num_targets
        blob_handle(i,:) = drawBlob([pixel_location(i,1), pixel_location(i,2)], pixel_location(i,3), []);        
    end
    xlabel('px (pixels)')
    ylabel('py (pixels)')
    grid on
else 
    for i=1:P.num_targets
        blob_handle(i,:) = drawBlob([pixel_location(i,1), pixel_location(i,2)], pixel_location(i,3), blob_handle(i,:));        
    end    
    hold on
end

% create output
temp_out = zeros(P.num_targets*3,1);
NN = 0;
for i=1:P.num_targets
    temp_out(1+NN) = pixel_location(i,1);
    temp_out(2+NN) = pixel_location(i,2);
    temp_out(3+NN) = pixel_location(i,3);
    NN = NN+3;
end
out = temp_out;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle=drawBlob(z, R, handle)

th = 0:.1:2*pi;
X = z(1)+ R*cos(th);
Y = z(2)+ R*sin(th);

if isempty(handle),
    handle = fill(X, Y, 'r');
else
    set(handle,'XData',X,'YData',Y);
end

end


%=======================================================================
% cameraProjection
% project the target onto the camera
%=======================================================================
function out = cameraProjection(tx,ty,tz,x,y,z,phi,theta,psi,P)

% vector pointing to the target from the camera in camera frame
vector_to_target = [tx; ty; tz] - [x; y; z];
vector_to_target = rotate(vector_to_target,-phi,-theta,-psi);   % (-) sign is added to phi, theta, psi to rotate the otherway in the camera FOV

if vector_to_target(3)<.1,  % if the target gets too close to the camera, the target disappears
    eps_x=-9999;
    eps_y=-9999;
    eps_size = 0;
else
    % These equations can be understood better with the UAV book Figure
    % 13.2. Think of similar triangles.
    eps_x =  P.f*(vector_to_target(1)/(vector_to_target(3)))+P.pixelnoise*randn;
    eps_y =  P.f*(vector_to_target(2)/(vector_to_target(3)))+P.pixelnoise*randn;
    eps_size = P.f*(P.target_size/(vector_to_target(3)))+P.pixelnoise*randn;
end

% snap output of camera to -9999 if outside field of view
tmp = P.cam_pix/2+eps_size;
if eps_x<-tmp | eps_x>tmp | eps_y<-tmp | eps_y>tmp,
    eps_x = -9999;
    eps_y = -9999;
    eps_size = 0;
end

out = [eps_x eps_y eps_size];
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
