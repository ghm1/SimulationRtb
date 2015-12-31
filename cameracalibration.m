%camera calibration
% addpath E:\Programme\RVC_toolboxes\robot-9.10\rvctools
% addpath E:\Programme\RVC_toolboxes\vision-3.4\rvctools
% addpath E:\Programme\RVC_toolboxes\contrib\rvctools\contrib\camera_calib
% addpath E:\Programme\RVC_toolboxes\contrib\rvctools\contrib\vgg
% startup_rvc

clear all
close all

P = mkcube(0.2);
T_unknown = transl(0.1, 0.2, 1.5) * rpy2tr(0.1, 0.2, 0.3);
cam = CentralCamera('focal', 0.015, 'pixel', 10e-6, ...
    'resolution', [1280 1024], 'centre', [512 512], ...
    'noise', 0.05);
p = cam.project(P, 'Tobj', T_unknown );
C = camcald(P, p)

est = invcamcal(C)
