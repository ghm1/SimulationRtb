%An example from Robotics, Vision and control by Peter Corke
%pbvs
%Attention: you have to define Tc first

addpath ..\RVC_toolboxes\contrib\rvctools\contrib\EPnP\EPnP
addpath ..\RVC_toolboxes\robot-9.10\rvctools
addpath ..\RVC_toolboxes\robust_pose_from_a_planar_target\rpp
addpath ..\RVC_toolboxes\robust_pose_from_a_planar_target\rpp\util
addpath ..\RVC_toolboxes\robust_pose_from_a_planar_target\rpp\objpose
startup_rvc

cam = CentralCamera('default');
P = mkgrid( 2, 0.5, 'T', transl(0,0,3) );
p = cam.plot(P, 'Tcam', Tc);
Tc_t_est = cam.estpose(P, p);
Tdelta = tcStar_t * inv(Tc_test);
Tdelta = trinterp(eye(4,4), Tdelta, lambda );
Tdelta = trnorm(Tc * Tdelta);
Tc0 = transl(1,1,-3) * trotz(0.6);
TcStar_t = transl(0, 0, 1);
pbvs = PBVS(cam, 'T0', Tc0, 'Tf', TcStar_t);
pbvs.run();
pbvs.plot_p();
pbvs.plot_vel();
pbvs.plot_camera();