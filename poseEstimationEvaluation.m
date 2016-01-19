%pose estimation evaluation

%comparison of of different pose estimation algorithms in accuracy and timing
clear
close all force

% addpath E:\Programme\RVC_toolboxes\contrib\rvctools\contrib\EPnP\EPnP
% addpath E:\Programme\RVC_toolboxes\robot-9.10\rvctools
% addpath robust_pose_from_a_planar_target\rpp\rpp
% addpath robust_pose_from_a_planar_target\rpp\rpp\util
% addpath robust_pose_from_a_planar_target\rpp\rpp\objpose
% startup_rvc

%definition of frames
%H_C_W = qlHomTrans(1, 1, 1, 180*pi/180, 10*pi/180, 0); % camera position wrt world frame
%H_T_W = qlHomTrans(1, 1, 0, 0, 0, 0);   % target position wrt world frame

%camera instance
cam = QLPerspectiveCamera(false);

if 1

    %position of local NED frame in worldframe
    H_LNED_W = [ 0  1  0  0;
                 1  0  0  0;
                 0  0 -1  0;
                 0  0  0  1];

    %position of body-NED frame in local NED frame          
    H_BFNED_LNED = [ 1  0  0  0;
                     0  1  0  0;
                     0  0  1  -7;
                     0  0  0  1];

    %position of camera in body-NED frame, defined by roll,pitch,yaw angles
    ax = 15; ay = -7; az = 2;
    % roll: -pi..pi, pitch -pi/2 .. pi, yaw: -pi .. pi
    roll = ax*pi/180; pitch = ay*pi/180; yaw = az*pi/180;
    R_C_BFNED = rotx(roll)*roty(pitch)*rotz(yaw);
    H_C_BFNED = [[ R_C_BFNED; zeros(1,3) ] [ 0; 0; 0; 1 ]];

    %position of camera in world frame
    H_C_W = H_LNED_W * H_BFNED_LNED * H_C_BFNED;
    
    cam.setRPY(roll, pitch, yaw);
    
    %target position in LNED: translation about 0.5 and -1.0 and rotation about
    %H_T_LNED = qlHomTrans(0.5, -1.0, 0, 0, 0, 2*pi)   % target position wrt world frame
%     H_T_LNED = qlHomTrans(0.5, -1.0, 0, 0, 0, 0)   % target position wrt world frame
%     H_T_W = H_LNED_W * H_T_LNED;
    H_T_W = qlHomTrans(0.5, -1.0, 0, 0, 0, 0);   % target position wrt world frame
else
    %definition of frames
    H_C_W = qlHomTrans(1, 1, 3, 180*pi/180, 0*pi/180, 0); % camera position wrt world frame
    H_T_W = qlHomTrans(1, 1, 0, 0, 0, 0);   % target position wrt world frame
end

%set camera position in wcs
cam.setH_C_W( H_C_W );
%set up target
opt.H_T_W = H_T_W;
opt.randomTarget = false;
opt.planar = true;
opt.nPts = 10;
opt.targetId = 1;
target = QLTarget(opt);

%project target onto cameras image plane
%target_I = cam.projectWorldPts(target.pts_W);

%plot
plotter = QLPlotter(cam, target);
%image noise
sigmaNoise = 1.0;
%flags if correspondence among points will be mixed
mixPts = false;
H_C_W_estStd_new = cam.estimatePose(target, 'Std', sigmaNoise, mixPts);
%H_C_W_estDLT_new = cam.estimatePose(target, 'EPnP', sigmaNoise, mixPts);
%H_C_W_estRPP_new = cam.estimatePose(target, 'RPP', sigmaNoise, mixPts);
%H_C_W_estRPP_new = cam.estimatePose(target, 'DLT', sigmaNoise, mixPts);

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
%     cam.estimatePose(target, 'RPP', sigmaNoise, false);
%     %wait a second
%     pause(1)
%     %save statistics: runtime, 
%     
%     k = k + dk;
% end












