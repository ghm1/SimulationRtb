%world to camera projection
% >> addpath E:\Programme\RVC_toolboxes\robot-9.10\rvctools
% >> addpath E:\Programme\RVC_toolboxes\vision-3.4\rvctools
% >> startup_rvc

%definition of camera object with 15mm focal lenght, 1mm pixel size
cam = CentralCamera('focal', 0.015);
%definition of testpoint in wcs
P = [0.2; 0.4; 3.0]
%projection of point into camera
cam.project(P)
%move camera 0.5 m to the left
tr = transl(-0.5, 0, 0)
cam.project(P, 'Tcam', tr)

%detailed camera
cam = CentralCamera('focal', 0.015, 'pixel', 10e-6, ...
    'resolution', [1280 1024], 'centre', [640 512], 'mycamera')
cam.project(P)
cam.C
cam.K

cam.plot(P)