function h = qlHomTrans( x, y, z, ax, ay, az)
%Build homogenious transformation matrix by definition of rotation angles

% Rotation matrix, model to camera 
Rx = [ 1 0 0; 0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)]; 
Ry = [ cos(ay) 0 sin(ay); 0 1 0; -sin(ay) 0 cos(ay)]; 
Rz = [ cos(az) -sin(az) 0; sin(az) cos(az) 0; 0 0 1]; 
R = Rz * Ry * Rx; 
%tansformation
transl = [x; y; z];

h = [R transl; 0 0 0 1];
return