%pose estimation evaluation

%comparison of DLT vs. EPnP in accuracy and timing
%comparison of usage of DLT or EPnP as seed for newton-gauss iterative optimization
clear all
close all

%definition of frames
H_C_W = qlHomTrans(2, 3, 10, 180*pi/180, 0, 0); % camera position wrt world frame
H_T_W = qlHomTrans(2, 3, 0, 0, 0, 85*pi/180);   % target position wrt world frame

%camera instance
cam = QLPerspectiveCamera();
%set camera position in wcs
cam.setH_C_W( H_C_W );

%target instance
target = QLTarget();
%set target position in wcs
target.setH_T_W( H_T_W );

%project target onto cameras image plane
target_I = cam.projectWorldPts(target.pts_W);

%plot (wcs is on (0,0,0))
qlPlotFrames( H_C_W, H_T_W );