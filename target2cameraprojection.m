%target to camera projection
clear all;
close all;

%-----------------------------------------------------------------------%
%Camera parameter:

%intrinsics:
%camera focal length in m
focal = 0.015;
%image sensor size in pixel
sensor_size_pix = [1280 1024];
%pixel size
pixel_size = 10e-6;
%principle point (here at the center of the sensor)
principle_pt = sensor_size_pix / 2;
%then the matrix K which combines the intrinsic parameters is:
K = [focal/pixel_size 0 principle_pt(1); ...
    0 focal/pixel_size principle_pt(2); ...
    0 0 1 ];

%position
t_C_W = [3; 2; 10];
H_C_W = homogeniousTransform( t_C_W, 180 * pi/180, 0, 0);

%extrinsic
H_W_C = inv(H_C_W);
Mext = H_W_C(1:3, :);
%-----------------------------------------------------------------------%
%definition of target in target frame T
ptDist = 1; % in m
P1 = [0; 0; 0; 1]; P2 = [ptDist; 0; 0; 1]; P3 = [0; ptDist; 0; 1]; P4 = [-ptDist; 0; 0; 1];
target_T = [P1 P2 P3 P4];

%transformation of target in world frame
%translation:
t_T_W = [2; 3; 0]; az = 85 * pi / 180;
%rotation about z-axis
H_T_W = homogeniousTransform( t_T_W, 0, 0, az);
target_W = H_T_W * target_T;
%the target is now in the world frame and we can project it into the camera

%-----------------------------------------------------------------------%
%target in world frame to camera projection
target_I = K * Mext * target_W;
target_I(1,:) = target_I(1,:)./target_I(3,:);
target_I(2,:) = target_I(2,:)./target_I(3,:);
target_I(3,:) = target_I(3,:)./target_I(3,:);

%-----------------------------------------------------------------------%
%-----------------------------------------------------------------------%
%visualize in world frame
%plot example point in world frame
hold on
grid on
for k=1:length(target_W)
    plot3(target_W(1,k), target_W(2,k), target_W(3,k),'.', 'Color', 'g');
end
%design axis to fit hole szene
axis ([0 6 0 6 0 10 0 1])

%plot world frame
xAxis_W = [1 0 0 0]
yAxis_W = [0 1 0 0]
zAxis_W = [0 0 1 0]
line([0 1], [0 0], [0 0], 'Color', 'r', 'LineWidth', 1.5);    % x axis
line([0 0], [0 1], [0 0], 'Color', 'g', 'LineWidth', 1.5);    % y axis
line([0 0], [0 0], [0 1], 'Color', 'b', 'LineWidth', 1.5);    % z axis
text(0,0,0,'W');

%plot camera frame (camera object only for visualization)
cam = CentralCamera('focal', focal, 'pixel', pixel_size, ...
    'resolution', sensor_size_pix, 'centre', [principle_pt(1) principle_pt(2)]);
cam.plot_camera('Tcam', H_C_W); %H_C_W is pose of camera with respect to the world frame
cam.plot(target_I(1:2,:), 'Tcam', H_C_W);

%-----------------------------------------------------------------------%
%-----------------------------------------------------------------------%
%poseestimation
%Make an initial guess of the pose [ax ay az tx ty tz] 
%In x are the transformation parameters to transform target points in
%target frame into camera frame -> H_T_C, that means the target positon
%with respect to the camera frame
x = [0.1; -0.1; 0.1; 2; 1; 5]; %[ax ay az tx ty tz] 
%estimate pose
x = poseEstimationNewton(x, target_I(1:2,:), target_T, K);
%build homogenious transform
H_T_C = homogeniousTransform([x(4); x(5); x(6)], x(1), x(2), x(3));
Mext_T = H_T_C(1:3,:);
% calculate reprojected point
target_I_rep = K * Mext_T * target_T;
target_I_rep(1,:) = target_I_rep(1,:)./target_I_rep(3,:);
target_I_rep(2,:) = target_I_rep(2,:)./target_I_rep(3,:);
target_I_rep(3,:) = target_I_rep(3,:)./target_I_rep(3,:);
%output result
disp(target_I_rep)
disp(target_I)
target_I_rep - target_I

%estimate position of camera
H_C_W_rep = H_T_W * inv(H_T_C);
disp(H_C_W_rep)
disp(H_C_W)

