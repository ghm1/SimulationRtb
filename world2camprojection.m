clear all;
close all;
%World to camerasensor transformation
%world frame is at (0,0,0)
addpath ..\RVC_toolboxes\robot-9.10\rvctools
addpath ..\RVC_toolboxes\robot-9.10\rvctools\robot
addpath ..\RVC_toolboxes\vision-3.4\rvctools
startup_rvc

%key:
% H = homogenious transformation
% P = homogeinous 3d point
% _C = camera frame
% _C_W = camera frame with respect to world frame

%an example point in worldframe (in homogenious coordinates)
Pw = [3; 2; 0; 1];
Pw_2 = [3; 3; 0; 1];
%where does this point project onto the camera sensor?

%-----------------------------------------------------------------------%
%camera parameters:

%extrinsics:
%first we have to define camera positon in world frame:
%(C_W means camera in world frame and can be used directly to transform points in
%camera frame to a point in the world frame)

%translation in inhomogenious coordinates
t_C_W = [3; 2; 10]; %camera origin in world frame
%rotation in inhomogenious coordinates
theta_x = 180;
rotX_C_W = [1 0 0; 0 cosd(theta_x) -sind(theta_x); 0 sind(theta_x) cosd(theta_x)]
theta_y = 0;
rotY_C_W = [cosd(theta_y) 0 sind(theta_y); 0 1 0; -sin(theta_y) 0 cos(theta_y)]
theta_z = 0;
rotZ_C_W = [cosd(theta_z) -sind(theta_z) 0; sind(theta_z) cosd(theta_z) 0; 0 0 1 ]

%concatenate rotations (beginning with the last rotation)
R_C_W = rotZ_C_W * rotY_C_W * rotX_C_W;
%concatenate rotation and translation to homogenious tranformation
H_C_W = [R_C_W t_C_W; 0 0 0 1];
%the transformation of a point from camera into the world frame is just the inverse of
%the transformation from camera to world
H_W_C_alt = inv(H_C_W);
%optimization:
%Because the inversion is computationally expensive we can optimize:
R_W_C = R_C_W'; %As the rotation is orthonormal, the transposed is the same as the inversed
t_W_C = -R_W_C * t_C_W; %the translation can be incersed with the negative roation matrix
H_W_C = [R_W_C t_W_C; 0 0 0 1];
%the output of isequal() shows that they are identical
transformation_equal = isequal(H_W_C_alt, H_W_C)

%intrinsics:
%camera focal length in m
focal = 0.015
%image sensor size in pixel
width_pix = 1280 %width in pixel
height_pix = 1024 %height in pixel
sensor_size_pix = [width_pix height_pix]
%pixel size
pixel_size = 10e-6;
%principle point (here at the center of the sensor)
principle_pt = sensor_size_pix / 2
%then the matrix K which combines the intrinsic parameters is:
K = [focal/pixel_size 0 principle_pt(1); ...
    0 focal/pixel_size principle_pt(2); ...
    0 0 1]

%-----------------------------------------------------------------------%
%transformation of the point in world frame to camera frame and back
Pc = H_W_C * Pw
Pw_back = H_C_W * Pc
transformation_equal = isequal(Pw, Pw_back)

%transformation of homogenious 3d point in camera frame to sensors pixel
%coordinate system
%1. projection to 2D homogenious coords
ProjMat = [1 0 0 0; 0 1 0 0; 0 0 1 0]
P_proj = ProjMat * Pc
%image point in homogenious coordinates
Pplane = K * P_proj
Pimg = [Pplane(1)/Pplane(3); Pplane(2)/Pplane(3)]

%now in one line
Pimg_alt = K * ProjMat * H_W_C * Pw
transformation_equal = isequal(Pimg_alt, Pplane)

%transform other points
C = K * ProjMat * H_W_C;
Pplane_2 = C * Pw_2
Pimg_2 = [Pplane_2(1)/Pplane_2(3); Pplane_2(2)/Pplane_2(3)]

%evaluation of toolbox function
cam = CentralCamera('focal', focal, 'pixel', pixel_size, ...
    'resolution', [width_pix height_pix], 'centre', [principle_pt(1) principle_pt(2)]);
Pimg_tb = cam.project(Pw, 'Tcam', H_C_W)
isequal(Pimg, Pimg_tb)
Pimg_2_tb = cam.project(Pw_2, 'Tcam', H_C_W)
isequal(Pimg_2, Pimg_2_tb)
%-----------------------------------------------------------------------%
%visualize in world frame
%plot example point in world frame
plot3(Pw(1), Pw(2), Pw(3),'.', 'Color', 'r');
hold on
grid on
plot3(Pw_2(1), Pw_2(2), Pw_2(3),'.', 'Color', 'g');
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
cam.plot_camera('Tcam', H_C_W); %H_C_W is pose of camera with respect to the world frame

%plot image plane
figure(2)
hold on 
grid on
axis ([0 sensor_size_pix(1) 0 sensor_size_pix(2)])
plot(Pimg(1), Pimg(2),'.', 'Color', 'r');
plot(Pimg_2(1), Pimg_2(2),'.', 'Color', 'g');
%and toolbox plot
cam.plot(Pimg, 'Tcam', H_C_W);
cam.plot(Pimg_2, 'Tcam', H_C_W);








