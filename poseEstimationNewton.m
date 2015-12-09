function x = poseEstimationNewton(x, target_I, target_T, K)
% poseEstimationNewton
% x: initial guess of the pose [ax ay az tx ty tz] 
% target_I: Target in target frame

%reshape
y0 = reshape(target_I, [], 1); % reshape into 2Nx1 vector 

for i=1:10 
    fprintf('\nIteration %d\nCurrent pose:\n', i); 
    disp(x); 
     
    %Get predicted image points:
    %projection of original target points onto image plane with actual
    %transformation parameters
    y = fProject(x, target_T, K); 
     
    % Estimate Jacobian 
    e = 0.00001;    % a tiny number 
    J(:,1) = ( fProject(x+[e;0;0;0;0;0],target_T,K) - y )/e; 
    J(:,2) = ( fProject(x+[0;e;0;0;0;0],target_T,K) - y )/e; 
    J(:,3) = ( fProject(x+[0;0;e;0;0;0],target_T,K) - y )/e; 
    J(:,4) = ( fProject(x+[0;0;0;e;0;0],target_T,K) - y )/e; 
    J(:,5) = ( fProject(x+[0;0;0;0;e;0],target_T,K) - y )/e; 
    J(:,6) = ( fProject(x+[0;0;0;0;0;e],target_T,K) - y )/e; 
    
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

return


