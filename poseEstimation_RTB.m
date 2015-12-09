%poseEstimation from Book Robotics and Computer vision
cam = CentralCamera('focal', 0.015, 'pixel',  10e-6, ...
    'resolution', [1280 1024], 'centre', [640 512]);

P = mkcube(0.2);

T_unknown = transl(0,0,2) * trotx(0.1) * troty(0.2)

p = cam.project(P, 'Tobj', T_unknown);

T_est = cam.estpose(P, p)