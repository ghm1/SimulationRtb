%pose estimation evaluation

%comparison of DLT vs. EPnP in accuracy and timing
%comparison of usage of DLT or EPnP as seed for newton-gauss iterative optimization
clear all
close all force

addpath E:\Programme\RVC_toolboxes\contrib\rvctools\contrib\EPnP\EPnP
addpath E:\Programme\RVC_toolboxes\robot-9.10\rvctools
%startup_rvc

%definition of frames
H_C_W = qlHomTrans(2, 3, 4, 180*pi/180, 0, 0); % camera position wrt world frame
H_T_W = qlHomTrans(2, 3, 0, 0, 0, 85*pi/180);   % target position wrt world frame

%camera instance
cam = QLPerspectiveCamera();
%set camera position in wcs
cam.setH_C_W( H_C_W );

%target instance
opt.H_T_W = H_T_W;
opt.randomTarget = true;
opt.planar = false;
opt.nPts = 10;
opt.targetId = 0;
target = QLTarget(opt);

%project target onto cameras image plane
%target_I = cam.projectWorldPts(target.pts_W);

%plot
plotter = QLPlotter(cam, target);

sigmaNoise = 0.0;
%H_C_W_estDLT_new = cam.estimatePose(target, 'DLT_GN', sigmaNoise);
%H_C_W_estRPP_new = cam.estimatePose(target, 'RPP', sigmaNoise);

%animation
%definition of start- and endtransformation
H_C_W_start = qlHomTrans(1, 2, 4, 180*pi/180, 0, 0); % camera position wrt world frame
H_C_W_end = qlHomTrans(3, 4, 4, 180*pi/180, 0, 0); % camera position wrt world frame

k = 0;
dk = 0.1;
while k < 1
    %interpolation between start- and endtransformation
    H_C_W = trinterp(H_C_W_start, H_C_W_end, k);
    %set camera position in wcs
    cam.setH_C_W( H_C_W );
    cam.estimatePose(target, 'EPnP_GN', sigmaNoise);
    %wait a second
    pause(1)
    %save statistics: runtime, 
    
    k = k + dk;
end












