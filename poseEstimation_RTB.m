

%poseEstimation from Book Robotics and Computer vision (S.266)
cam = CentralCamera('focal', 0.015, 'pixel',  10e-6, ...
    'resolution', [1280 1024], 'centre', [640 512]);
%Targetpunkte im WCS generieren: 8 Punkte im Abstand von 0.2 zueinander auf
%dem ursprung des wcs (Target-CS und WCS liegen aufeinander, deshalb ist H_T_C == H_W_C)
P = mkcube(0.2);
plot_sphere(P, 0.05, 'b');

%Transformation der 
T_unknown = transl(0,0,2) * trotx(0.1) * troty(0.2)
%projektion der Punkte auf die Sensorebene
p = cam.project(P, 'Tobj', T_unknown);
P = homtrans(T_unknown, P);
%bestimmung von H_T_C, ist hier gleich H_W_C
T_est = cam.estpose(P, p)


hold on
cam.plot_camera('Tcam', T_est);
