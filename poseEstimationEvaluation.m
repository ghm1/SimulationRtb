%pose estimation evaluation

%comparison of DLT vs. EPnP in accuracy and timing
%comparison of usage of DLT or EPnP as seed for newton-gauss iterative optimization
clear
close all force

% addpath E:\Programme\RVC_toolboxes\contrib\rvctools\contrib\EPnP\EPnP
% addpath E:\Programme\RVC_toolboxes\robot-9.10\rvctools
% addpath robust_pose_from_a_planar_target\rpp\rpp
% addpath robust_pose_from_a_planar_target\rpp\rpp\util
% addpath robust_pose_from_a_planar_target\rpp\rpp\objpose
% startup_rvc

%definition of frames
%H_C_W = qlHomTrans(1, 3.5, 4, 180*pi/180, 10*pi/180, 0); % camera position wrt world frame
H_T_W = qlHomTrans(0, 0, 0, 0, 0, 0);   % target position wrt world frame

H_LNED_W = [ 0  1  0  0;
             1  0  0  0
             0  0 -1  0
             0  0  0  1];
         
H_BFNED_W = [ 0  1  0  0;
              1  0  0  0
              0  0 -1  4
              0  0  0  1];

%add original roll, pitch and yaw to BFNED
roll = 0; pitch = 0*pi/180; yaw = 0;
H_C_BFNED = rpy2tr(roll, pitch, yaw);
%test
H_C_W = H_BFNED_W * H_C_BFNED;

%camera instance
cam = QLPerspectiveCamera();
%set camera position in wcs
cam.setH_C_W( H_C_W );
cam.setRPY(roll, pitch, yaw);

%target instance
opt.H_T_W = H_T_W;
opt.randomTarget = false;
opt.planar = true;
opt.nPts = 6;
opt.targetId = 1;
target = QLTarget(opt);

%project target onto cameras image plane
%target_I = cam.projectWorldPts(target.pts_W);

%plot
plotter = QLPlotter(cam, target);

sigmaNoise = 0.0;
mixPts = false;
H_C_W_estStd_new = cam.estimatePose(target, 'Std', sigmaNoise, mixPts);
%H_C_W_estDLT_new = cam.estimatePose(target, 'EPnP', sigmaNoise);
%H_C_W_estRPP_new = cam.estimatePose(target, 'RPP', sigmaNoise);

% %animation
% %definition of start- and endtransformation
% H_C_W_start = qlHomTrans(1, 2, 4, 180*pi/180, 0, 0); % camera position wrt world frame
% H_C_W_end = qlHomTrans(3, 4, 4, 180*pi/180, 0, 0); % camera position wrt world frame
% 
% k = 0;
% dk = 0.1;
% while k < 1
%     %interpolation between start- and endtransformation
%     H_C_W = trinterp(H_C_W_start, H_C_W_end, k);
%     %set camera position in wcs
%     cam.setH_C_W( H_C_W );
%     cam.estimatePose(target, 'EPnP', sigmaNoise);
%     %wait a second
%     pause(1)
%     %save statistics: runtime, 
%     
%     k = k + dk;
% end












