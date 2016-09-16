%homography
clear; % all;
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

%example points in worldframe (in homogenious coordinates)
Pw = [3; 2; 0; 1];
Pw_2 = [3; 3; 0; 1];
pts_W = [Pw Pw_2];
%-----------------------------------------------------------------------%
%camera parameters:

%extrinsics:
%first we have to define camera positon in world frame:
%(C_W means camera in world frame and can be used directly to transform points in
%camera frame to a point in the world frame)

%translation in inhomogenious coordinates
t_C_W = [3; 2; 10]; %camera origin in world frame
%rotation in inhomogenious coordinates
theta_z = 180;
H_C_W = homogeniousTransform( t_C_W, 0, 0, theta_z * pi/180);
H_W_C = inv(H_C_W);

%intrinsics:
%camera focal length in m
focal = 0.015;
%image sensor size in pixel
width_pix = 1280; %width in pixel
height_pix = 1024; %height in pixel
sensor_size_pix = [width_pix height_pix];
%pixel size
pixel_size = 10e-6;
%principle point (here at the center of the sensor)
principle_pt = sensor_size_pix / 2;
%then the matrix K which combines the intrinsic parameters is:
K = [focal/pixel_size 0 principle_pt(1); ...
    0 focal/pixel_size principle_pt(2); ...
    0 0 1];

%-----------------------------------------------------------------------%
%transformation of homogenious 3d point in camera frame to sensors pixel
%coordinate system
%1. projection to 2D homogenious coords
ProjMat = [1 0 0 0; 0 1 0 0; 0 0 1 0];
pts_I1 = K * ProjMat * H_W_C * pts_W;
[M,N] = size(pts_I1);
ptsImg = zeros(2,N);
ptsImg(1,:) = pts_I1(1,:)./pts_I1(3,:);
ptsImg(2,:) = pts_I1(2,:)./pts_I1(3,:);

%-----------------------------------------------------------------------%
%homographie: projection of homogenous image pts in rotated camera to image
%pts in orthogonal camera 
%roll, pitch and yaw are rotation angles from orthogonal camera (C2) to
%rotated camera (C1)
roll = 10; pitch = 20; yaw = 0;
R_C2_C1 = rpy2r(roll, pitch, yaw);
R_C1_C2 = R_C2_C1'; %as the rotationmatix is orthonormal, the transposed is the same as the inverse
pts_I2 = K * R_C1_C2 / K * pts_I1; % R_C1_C2 / K  means R_C1_C2 * inv(K)













