%pose estimation evaluation

%comparison of DLT vs. EPnP in accuracy and timing
%comparison of usage of DLT or EPnP as seed for newton-gauss iterative optimization
clear all
close all force

addpath E:\Programme\RVC_toolboxes\contrib\rvctools\contrib\EPnP\EPnP

%definition of frames
H_C_W = qlHomTrans(2, 3, 4, 180*pi/180, 0, 0); % camera position wrt world frame
H_T_W = qlHomTrans(2, 3, 0, 0, 0, 85*pi/180);   % target position wrt world frame

%camera instance
cam = QLPerspectiveCamera();
%set camera position in wcs
cam.setH_C_W( H_C_W );

%target instance
target = QLTarget(2);
target.setH_T_W(H_T_W);

%project target onto cameras image plane
target_I = cam.projectWorldPts(target.pts_W);

%plot
plotter = QLPlotter(cam, target);

sigmaNoise = 0.5;
H_C_W_estDLT_new = cam.estimatePose(target, 'DLT_GN', sigmaNoise);
% H_C_W_estDLT_GN_new = cam.estimatePose(target, 'DLT_GN', sigmaNoise);
% H_C_W_estEPnP_new = cam.estimatePose(target, 'EPnP', sigmaNoise);

% %Plot projected image pts, projected image pts plus noise and reprojected
% %pts
% I = zeros(cam.sensorSize(2),cam.sensorSize(1)); 
% figure
% imshow(I); 
% hold on 
% %projected image pts
% plot(target_I(1,:), target_I(2,:), 'g*');
% %projected image pts plus noise
% % Add some noise to the image points 

% N = length( target_I(1,:));
% target_I_noise(1:2,:) = target_I(1:2,:) + sigmaNoise*randn(2,N); 
% target_I_noise = [target_I_noise; ones(1,N)];
% plot(target_I_noise(1,:), target_I_noise(2,:), 'w*'); 

% %position estimation
% pe = QLPoseEstimation();
% %For DLT we need minumum 6 points because the equation to solve has 6
% %unknowns
% H_C_W_estDLT = pe.estPoseDLT(cam.K, target.pts_W, target_I_noise );
% H_W_C_estDLT = inv(H_C_W_estDLT);
% Mext_estDLT = H_W_C_estDLT( 1:3, : );
% target_I_rep_DLT = cam.K * Mext_estDLT * target.pts_W;
% target_I_rep_DLT(1,:) = target_I_rep_DLT(1,:) ./ target_I_rep_DLT(3,:);
% target_I_rep_DLT(2,:) = target_I_rep_DLT(2,:) ./ target_I_rep_DLT(3,:);
% target_I_rep_DLT(3,:) = target_I_rep_DLT(3,:) ./ target_I_rep_DLT(3,:);
% %plot
% plot(target_I_rep_DLT(1,:), target_I_rep_DLT(2,:), 'r*'); 
% 
% %optimize pose with gauss-newton optimization
% H_C_W_estGN = pe.estPoseGN( cam.K, target.pts_W, target_I_noise, H_C_W_estDLT );
% %reproject image points using H_C_W_estGN
% H_W_C_estGN = inv(H_C_W_estGN);
% Mext_estGN = H_W_C_estGN( 1:3, : );
% target_I_rep_GN = cam.K * Mext_estGN * target.pts_W;
% target_I_rep_GN(1,:) = target_I_rep_GN(1,:) ./ target_I_rep_GN(3,:);
% target_I_rep_GN(2,:) = target_I_rep_GN(2,:) ./ target_I_rep_GN(3,:);
% target_I_rep_GN(3,:) = target_I_rep_GN(3,:) ./ target_I_rep_GN(3,:);
% %plot
% plot(target_I_rep_GN(1,:), target_I_rep_GN(2,:), 'y*'); 
% 
% %estimate pose with EPnP Algorithm
% H_C_W_estEPnP = pe.estPoseEPnP(cam.K, target.pts_W, target_I_noise );
% %reproject image points using H_C_W_estEPnP
% H_W_C_estEPnP = inv(H_C_W_estEPnP);
% Mext_estEPnP = H_W_C_estEPnP( 1:3, : );
% target_I_rep_EPnP = cam.K * Mext_estEPnP * target.pts_W;
% target_I_rep_EPnP(1,:) = target_I_rep_EPnP(1,:) ./ target_I_rep_EPnP(3,:);
% target_I_rep_EPnP(2,:) = target_I_rep_EPnP(2,:) ./ target_I_rep_EPnP(3,:);
% target_I_rep_EPnP(3,:) = target_I_rep_EPnP(3,:) ./ target_I_rep_EPnP(3,:);
% %plot
% plot(target_I_rep_EPnP(1,:), target_I_rep_GN(2,:), 'b*'); 












