% DLT algorithm (direct linear transform) 
clear all 
close all 
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Create input data 
  
% Create camera matrix 
f = 512;      % focal length in pixels 
cx = 256; 
cy = 256; 
K = [ f 0 cx; 0 f cy; 0 0 1 ];   % intrinsic parameter matrix 
  
N = 8;  % Create known 3D points (at least 6) 
% P_M = [ 
%     rand(3,N)-0.5;  % Points within a cube of unit length 
%     ones(1,N) 
%     ]; 
P_M = [ 
    1   -1  1   -1  1   -1  1   -1;     % points on a cube 
    1   1   -1  -1  1   1   -1  -1; 
    1   1   1   1   -1  -1  -1  -1; 
    1   1   1   1   1   1   1   1]; 

% Create true model-to-camera transform 
ax = 0; ay = 20*pi/180; az = -30*pi/180; 
tx = 0; ty = 0; tz = 6; 
Rx = [ 1 0 0; 0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)]; 
Ry = [ cos(ay) 0 sin(ay); 0 1 0; -sin(ay) 0 cos(ay)]; 
Rz = [ cos(az) -sin(az) 0; sin(az) cos(az) 0; 0 0 1]; 
R_w_c = Rz * Ry * Rx; 
H_w_c = [R_w_c [tx;ty;tz]; 0 0 0 1]; 
disp('Ground truth pose, model to camera:'); disp(H_w_c); 
  
H_c_w = inv(H_w_c); 
disp('Ground truth pose, camera to model:'); disp(H_c_w); 
  
% Project points onto image 
Mext = H_w_c(1:3, :);       % Camera extrinsic matrix 
p = K*Mext*P_M; 
p(1,:) = p(1,:)./p(3,:); 
p(2,:) = p(2,:)./p(3,:); 
p(3,:) = p(3,:)./p(3,:); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Display input data 
disp('Known model points:'); disp(P_M); 
disp('Measured image points:'); disp(p); 
I = zeros(512,512); 
imshow(I); 
hold on 
plot(p(1,:), p(2,:), 'g*'); 
  
% Add some noise to the image points 
sigma = 5.0; 
p(1:2,:) = p(1:2,:) + sigma*randn(2,N); 
plot(p(1,:), p(2,:), 'w*'); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Solve for the pose of the model with respect to the camera. 
  
pn = inv(K)*p;  % Normalize image points 
  
% Ok, now we have pn = Mext*P_M. 
% If we know P_M and pn, we can solve for the elements of Mext. 
% The equations for x,y are: 
%   x = (r11*X + r12*Y + r13*Z + tx)/(r31*X + r32*Y + r33*Z + tz) 
%   y = (r21*X + r22*Y + r23*Z + ty)/(r31*X + r32*Y + r33*Z + tz) 
% or 
%   r11*X + r12*Y + r13*Z + tx - x*r31*X - x*r32*Y - x*r33*Z - x*tz = 0 
%   r21*X + r22*Y + r23*Z + ty - y*r31*X - y*r32*Y - y*r33*Z - y*tz = 0 
  
% Put elements of Mext into vector w: 
%   w = [r11 r12 r13 r21 r22 r23 r31 r32 r33 tx ty tz] 
% We then have Ax = 0.  The rows of A are: 
%   X  Y  Z  0  0  0  -x*X  -x*Y  -x*Z  1  0  -x 
%   0  0  0  X  Y  Z  -y*X  -y*Y  -y*Z  1  0  -y 
  
A = zeros(N,12); 
for i=1:N 
    X = P_M(1,i);   Y = P_M(2,i);   Z = P_M(3,i); 
    x = pn(1,i);    y = pn(2,i); 
    A( 2*(i-1)+1, :) = [ X  Y  Z  0  0  0  -x*X  -x*Y  -x*Z  1  0  -x ]; 
    A( 2*(i-1)+2, :) = [ 0  0  0  X  Y  Z  -y*X  -y*Y  -y*Z  0  1  -y ]; 
end 


% Solve for the value of x that satisfies Ax = 0. 
% The solution to Ax=0 is the singular vector of A corresponding to the 
% smallest singular value; that is, the last column of V in A=UDV' 
[U,D,V] = svd(A); 
x = V(:,end);                  % get last column of V 
  
% Reshape x back to a 3x4 matrix, M = [R  t] 
M = [ x(1)  x(2)  x(3)  x(10); 
      x(4)  x(5)  x(6)  x(11); 
      x(7)  x(8)  x(9)  x(12) ]; 
  
% We can find the camera center, tcorg_w by solving the equation MX=0. 
% To see this, write M = [R_w_c tmorg_c].  But tmorg_c = -R_w_c * tcorg_w. 
% So M = R_w_c*[ I  -tcorg_w ].  And if we multiply M times tcorg_w, we 
% get   R_w_c*[ I  -tcorg_w ] * [tcorg_w; 1] = 0. 
[U,D,V] = svd(M); 
tcorg_w = V(:,end);     % Get last column of V 
tcorg_w = tcorg_w / tcorg_w(4);     % Divide through by last element  

% Get rotation portion from M 
[Q,B] = qr(M(1:3,1:3)'); 
  
% Enforce that the diagonal values of B are positive 
for i=1:3 
    if B(i,i)<0 
        B(i,:) = -B(i,:);   % Change sign of row 
        Q(:,i) = -Q(:,i);   % Change sign of column 
    end 
end 
         
Restimated_w_c = Q';     % Estimated rotation matrix, model-to-camera 
  
% R must be a right handed rotation matrix; ie det(R)>0 
if det(Restimated_w_c)<0 
    Restimated_w_c = -Restimated_w_c; 
end 
  
% Final estimated pose 
Restimated_c_w = Restimated_w_c'; 
Hestimated_c_w = [Restimated_c_w tcorg_w(1:3); 0 0 0 1]; 
disp('Final computed pose, H_c_w:'), disp(Hestimated_c_w); 
  
Hestimated_w_c = inv(Hestimated_c_w); 
disp('Final computed pose, H_w_c:'), disp(Hestimated_w_c);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Reproject points back onto the image 
M = Hestimated_w_c(1:3,:); 
p = K*M*P_M; 
p(1,:) = p(1,:)./p(3,:); 
p(2,:) = p(2,:)./p(3,:); 
p(3,:) = p(3,:)./p(3,:); 
plot(p(1,:), p(2,:), 'r*');
