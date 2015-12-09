%pbvs
clear all
close all

%world to camera projection
% >> addpath E:\Programme\RVC_toolboxes\robot-9.10\rvctools
% >> addpath E:\Programme\RVC_toolboxes\robot-9.10\rvctools\robot
% >> addpath E:\Programme\RVC_toolboxes\vision-3.4\rvctools
% >> startup_rvc

% cam = CentralCamera('default');
% P = mkgrid( 2, 0.5, 'T', transl(0,0,3) );
% Tc_unknown = transl(2, 2, -4) * trotz(0.2);
% cam.plot_camera('Tcam', Tc_unknown);
% p = cam.plot(P, 'Tcam', Tc_unknown);
% %check, if projection lies in camera fov
% if ~isnan(p)
%     Tc_t_est = cam.estpose(P, p);
%     %required target position with respect to camera frame
%     tcStar_t = transl(0, 0, 1);
%     Tdelta = tcStar_t * inv(Tc_t_est);
%     lambda = 0.1;
%     Tdelta = trinterp(eye(4,4), Tdelta, lambda );
%     Tdelta = trnorm(Tc * Tdelta);
%     Tc0 = transl(1,1,-3) * trotz(0.6);
%     TcStar_t = transl(0, 0, 1);
%     pbvs = PBVS(cam, 'T0', Tc0, 'Tf', TcStar_t);
%     pbvs.run();
%     pbvs.plot_p();
%     pbvs.plot_vel();
%     pbvs.plot_camera();
% else
%     disp('projection not on image plane')
% end

cam = CentralCamera('default');
Tc0 = transl(1,1,-3) * trotz(0.6);
TcStar_t = transl(0,0,1);
pbvs = PBVS(cam, 'T0', Tc0, 'Tf', TcStar_t);
pbvs.run();
pbvs.plot_p();
pbvs.plot_vel();
pbvs.plot_camera();