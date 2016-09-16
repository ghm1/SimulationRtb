%transformation of a point from one camera to another
clear
close all force

addpath ..\RVC_toolboxes\contrib\rvctools\contrib\EPnP\EPnP
addpath ..\RVC_toolboxes\robot-9.10\rvctools
addpath ..\RVC_toolboxes\robust_pose_from_a_planar_target\rpp
addpath ..\RVC_toolboxes\robust_pose_from_a_planar_target\rpp\util
addpath ..\RVC_toolboxes\robust_pose_from_a_planar_target\rpp\objpose
startup_rvc

%key:
% H = homogenious transformation
% P = homogeinous 3d point
% _C = camera frame
% _C_W = camera frame with respect to world frame

%--------------------------------------------------------------------------
% Definition of camera 1
camParams1.name = 'cam1';    % camera name
%             obj.f    = 0.015;    % focal length
%             obj.sensorSize = [1280 1024];  % number of pixel 1x2
%             obj.pp = obj.sensorSize/2;      % principal point 1x2
%             obj.rho = 10e-6;

camParams1.f    = [ 323.70584 324.26201 ];    % focal length (f/rho)
camParams1.sensorSize = [158.5*2 98.5*2];  % number of pixel 1x2
camParams1.pp = [160.44416 100.75304];      % principal point 1x2
camParams1.rho = 1;

%distortion parameter
camParams1.k1 = -0.43772;
camParams1.k2 = 0.25627;
camParams1.p1 = -0.00005;
camParams1.p2 = -0.00102;
camParams1.k3 = 0.0;
camParams1.distort = true; %apply distortion

cam1 = QLPerspectiveCamera(camParams1);
%--------------------------------------------------------------------------
% Definition of camera 2
camParams2.name = 'cam1';    % camera name
%             obj.f    = 0.015;    % focal length
%             obj.sensorSize = [1280 1024];  % number of pixel 1x2
%             obj.pp = obj.sensorSize/2;      % principal point 1x2
%             obj.rho = 10e-6;

camParams2.f    = [ 323.70584 324.26201 ];    % focal length (fx/rho)
camParams2.sensorSize = [158.5*2 98.5*2];  % number of pixel 1x2
camParams2.pp = [160.44416 100.75304];      % principal point 1x2
camParams2.rho = 1;

%distortion parameter
camParams2.k1 = -0.43772;
camParams2.k2 = 0.25627;
camParams2.p1 = -0.00005;
camParams2.p2 = -0.00102;
camParams2.k3 = 0.0;
camParams2.distort = true; %apply distortion

cam2 = QLPerspectiveCamera(camParams2);
%--------------------------------------------------------------------------
% Definition of position of camera 1
% translation
transl = [ 1; 1; 1 ];
%Transformation of camera with respect to world frame
H1_C_W = homogeniousTransform( transl, 0, 0, 0 );
%set camera position in wcs
cam1.setH_C_W( H1_C_W );
%--------------------------------------------------------------------------
% Definition of position of camera 2
% translation
transl = [ 0; 0; 1 ];
%Transformation of camera with respect to world frame
H1_C_W = homogeniousTransform( transl, 0, 0, 0 );
%set camera position in wcs
cam1.setH_C_W( H1_C_W );
%--------------------------------------------------------------------------
% Definition of points in world coordinate system

%--------------------------------------------------------------------------
% Projection of world points onto sensor of camera 1

%--------------------------------------------------------------------------
% Projection of world points onto sensor of camera 2

%--------------------------------------------------------------------------
% Projection of points on sensor of camera 1 onto sensor of camera 2


