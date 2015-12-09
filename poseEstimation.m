clear all  
close all 
  
I = imread('img1_rect.tif'); 
imshow(I, []) 
  
% These are the points in the model's coordinate system (inches) 
P_M = [  0      0       2       0       0       2; 
        10      2       0      10       2       0; 
         6      6       6       2       2       2; 
         1      1       1       1       1       1 ]; 
  
% Define camera parameters 
f = 715;      % focal length in pixels 
cx = 354; 
cy = 245; 
  
K = [ f 0 cx; 0 f cy; 0 0 1 ];   % intrinsic parameter matrix 
  
y0 = [ 183; 147;    % 1 
    350; 133;       % 2 
    454; 144;       % 3 
    176; 258;       % 4 
    339; 275;       % 5 
    444; 286 ];     % 6 
  
% Make an initial guess of the pose [ax ay az tx ty tz] 
x = [1.5; -1.0; 0.0; 0; 0; 30]; 
  
% Get predicted image points by substituting in the current pose 
y = fProject(x, P_M, K); 
 
for i=1:10 
    fprintf('\nIteration %d\nCurrent pose:\n', i); 
    disp(x); 
     
    % Get predicted image points  
    y = fProject(x, P_M, K); 
  
    %imshow(I, []) 
    %for i=1:2:length(y) 
    %    rectangle('Position', [y(i)-8 y(i+1)-8 16 16], ... 
    %        'FaceColor', 'r'); 
    %end 
    %pause(1); 
     
    % Estimate Jacobian 
    e = 0.00001;    % a tiny number 
    J(:,1) = ( fProject(x+[e;0;0;0;0;0],P_M,K) - y )/e; 
    J(:,2) = ( fProject(x+[0;e;0;0;0;0],P_M,K) - y )/e; 
    J(:,3) = ( fProject(x+[0;0;e;0;0;0],P_M,K) - y )/e; 
    J(:,4) = ( fProject(x+[0;0;0;e;0;0],P_M,K) - y )/e; 
    J(:,5) = ( fProject(x+[0;0;0;0;e;0],P_M,K) - y )/e; 
    J(:,6) = ( fProject(x+[0;0;0;0;0;e],P_M,K) - y )/e; 
  
  
    % Error is observed image points - predicted image points 
    dy = y0 - y; 
    fprintf('Residual error: %f\n', norm(dy)); 
  
    % Ok, now we have a system of linear equations   dy = J dx 
    % Solve for dx using the pseudo inverse 
    dx = pinv(J) * dy; 
  
    % Stop if parameters are no longer changing 
    if abs( norm(dx)/norm(x) ) < 1e-6 
        break; 
    end 
  
    x = x + dx;   % Update pose estimate 
end 

imshow(I, []) 
for i=1:2:length(y) 
    rectangle('Position', [y(i)-8 y(i+1)-8 16 16], ... 
        'FaceColor', 'r'); 
end 
pause(1); 

% Draw coordinate axes onto the image.  Scale the length of the axes 
% according to the size of the model, so that the axes are visible. 
W = max(P_M,[],2) - min(P_M,[],2);  % Size of model in X,Y,Z 
W = norm(W);   % Length of the diagonal of the bounding box 
  
u0 = fProject(x, [0;0;0;1], K);  % origin 
uX = fProject(x, [W/5;0;0;1], K);  % unit X vector 
uY = fProject(x, [0;W/5;0;1], K);  % unit Y vector 
uZ = fProject(x, [0;0;W/5;1], K);  % unit Z vector 
  
line([u0(1) uX(1)], [u0(2) uX(2)], 'Color', 'r', 'LineWidth', 3); 
line([u0(1) uY(1)], [u0(2) uY(2)], 'Color', 'g', 'LineWidth', 3); 
line([u0(1) uZ(1)], [u0(2) uZ(2)], 'Color', 'b', 'LineWidth', 3); 
 
% Also print the pose onto the image. 
text(30,450,sprintf('ax=%.2f ay=%.2f az=%.2f tx=%.1f ty=%.1f tz=%.1f', ... 
    x(1), x(2), x(3), x(4), x(5), x(6)), ... 
    'BackgroundColor', 'w', 'FontSize', 15); 




