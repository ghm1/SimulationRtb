%indecies
R = [1 2; 3 4]
R_n = [R(1,1) R(1,2); R(2,1) R(2,2)]
%2D transformations
%rotation
v = [4; 5] %vector
theta = pi; % rotation matrix
R = [cos(theta) -sin(theta); sin(theta) cos(theta)]
v_n = R * v
%plot two vectors
plotv([v(1) v_n(1); v(2) v_n(2)], '-');

%example 2
x_t = [10; 20; 1]
theta = 45*pi/180 % degree
R = [cos(theta) -sin(theta); sin(theta) cos(theta)]
H = [R(1,1) R(1,2) 40; R(2,1) R(2,2) -30; 0 0 1 ]
x_t_strich = H * x_t

%3d plot
a = [2 3 5];
b = [1 1 0];
c = a+b;
starts = zeros(3,3);
ends = [a;b;c];
quiver3(starts(:,1), starts(:,2), starts(:,3), ends(:,1), ends(:,2), ends(:,3))
axis equal

v = [1; 1; 1];
%position x,y,z of vector and vector components
quiver3(0, 0, 0, v(1), v(2), v(3));
axis equal

%3D transformations
ax = 0.1; ay = -0.2; az = 0.3 % radians
Rx = [1 0 0; 0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)];
Ry = [ cos(ay) 0 sin(ay); 0 1 0; -sin(ay) 0 cos(ay)];
Rz = [ cos(az) -sin(az) 0; sin(az) cos(az) 0; 0 0 1];
R = Rz * Ry * Rx
R = Rx * Ry * Rz

%homogenious 3d transformation
P_A = [1; 0; 1; 1];
T = [0;0;10];
theta = pi;
Rx = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
H_A_B = [Rx T; 0 0 0 1] %transformation from A to B
H_B_A = inv(H_A_B);
P_B = H_B_A * P_A

%world to cam transformation exercise
K = [512 0 256; 0 512 256; 0 0 1]
H_C_W = [1 0 0 5; 0 -1 0 0; 0 0 -1 1; 0 0 0 1]*[0 0 1 1; 1 0 0 0; 0 1 0 -2; 0 0 0 1]%transformation from cam to world
H_W_C = inv(H_C_W)
M_ext = H_W_C(1:3, :) %indexzugriff: zeilen 1 bis 3 und alle spalten
x_w = [16;0;-1;1]
x_c = K * M_ext * x_w
x_im = x_c(1)/x_c(3)
y_im = x_c(2)/x_c(3)
