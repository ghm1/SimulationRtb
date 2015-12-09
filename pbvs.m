%pbvs
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