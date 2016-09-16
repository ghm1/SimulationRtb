%pose estimation evaluation

%comparison of of different pose estimation algorithms in accuracy and timing
clear
close all force

addpath ..\RVC_toolboxes\contrib\rvctools\contrib\EPnP\EPnP
addpath ..\RVC_toolboxes\robot-9.10\rvctools
addpath ..\RVC_toolboxes\robust_pose_from_a_planar_target\rpp
addpath ..\RVC_toolboxes\robust_pose_from_a_planar_target\rpp\util
addpath ..\RVC_toolboxes\robust_pose_from_a_planar_target\rpp\objpose
startup_rvc

%key:
% H = homogenious transformation
% P = homogeinous 3d point
% _C = camera frame
% _C_W = camera frame with respect to world frame

%camera position
cameraHeigth = 2;
cameraX = -2;
cameraY = -1;
roll = 0; pitch = 0; yaw = 0;
%target position;
targetX = 0.0;
targetY = 0.0;
targetRotZGrad = 0.0;
%image noise
sigmaNoise = 0.5;

%algorithm
algoTypes = {'Std' 'RPP'}; %'RPP' 'EPnP' 'Std'
targetIds = [1 5];
%target parameter
random = false; %flags, if the target should be random generated (this beats the target id)
planar = true; %flags, if the target should be planar (random case)
nPts = 10;     %number of points in target (random case)
targetId = 1; %id of fixed target (only works, when ramdom = false)

showPlot = true;
%error evaluation
M = 2; %number of Algorithms
N = 10; %number of iterations
x_y_error = zeros(M,N);
z_error = zeros(M,N);

%--------------------------------------------------------------------------
camParams1.name = 'cam1';    % camera name
%             obj.f    = 0.015;    % focal length
%             obj.sensorSize = [1280 1024];  % number of pixel 1x2
%             obj.pp = obj.sensorSize/2;      % principal point 1x2
%             obj.rho = 10e-6;

camParams1.f    = [ 323.70584 324.26201 ];    % focal length (fx/rho)
camParams1.sensorSize = [158.5*2 98.5*2];  % number of pixel 1x2
camParams1.pp = [160.44416 100.75304];      % principal point 1x2
camParams1.rho = 1;

%distortion parameter
camParams1.k1 = -0.43772;
camParams1.k2 = 0.25627;
camParams1.p1 = -0.00005;
camParams1.p2 = -0.00102;
camParams1.k3 = 0.0;
camParams1.distort = true; %apply distortion
%--------------------------------------------------------------------------

for l=1 : 2
    
    for k=1 : N
        %if error estimation active, generate random Positions
        if N > 1    
            close all force
            cameraX = -1 + 2*rand(1);
            cameraY = -0.5 + 1*rand(1);
            roll = -5 + 5*rand(1);
            pitch = -5 + 5*rand(1);
            yaw = -90 + 180*rand(1);
        end

        %camera instance
        cam = QLPerspectiveCamera(camParams1);
        if 1

            %position of local NED frame in worldframe
            H_LNED_W = [ 0  1  0  0;
                         1  0  0  0;
                         0  0 -1  0;
                         0  0  0  1];

            %position of body-NED frame in local NED frame          
            H_BFNED_LNED = [ 1  0  0  cameraX;
                             0  1  0  cameraY;
                             0  0  1  -cameraHeigth;
                             0  0  0  1];

            %position of camera in body-NED frame, defined by roll,pitch,yaw angles
            % roll: -pi..pi, pitch -pi/2 .. pi, yaw: -pi .. pi
            roll = roll*pi/180; pitch = pitch*pi/180; yaw = yaw*pi/180;
            R_C_BFNED = rotx(roll)*roty(pitch)*rotz(yaw);
            H_C_BFNED = [[ R_C_BFNED; zeros(1,3) ] [ 0; 0; 0; 1 ]];

            %position of camera in world frame
            H_C_W = H_LNED_W * H_BFNED_LNED * H_C_BFNED;

            cam.setRPY(roll, pitch, yaw);

            %target position in LNED: translation about 0.5 and -1.0 and rotation about
            %H_T_LNED = qlHomTrans(0.5, -1.0, 0, 0, 0, 2*pi)   % target position wrt world frame
        %     H_T_LNED = qlHomTrans(0.5, -1.0, 0, 0, 0, 0)   % target position wrt world frame
        %     H_T_W = H_LNED_W * H_T_LNED;
            H_T_W = qlHomTrans(targetX, targetY, 0, 0, 0, targetRotZGrad * pi/180);   % target position wrt world frame
        else
            %definition of frames
            H_C_W = qlHomTrans(1, 1, 3, 180*pi/180, 0*pi/180, 0); % camera position wrt world frame
            H_T_W = qlHomTrans(1, 1, 0, 0, 0, 0);   % target position wrt world frame
        end

        %set camera position in wcs
        cam.setH_C_W( H_C_W );

        %set up target
        opt.H_T_W = H_T_W;
        opt.randomTarget = random;
        opt.planar = planar;
        opt.nPts = nPts;
        opt.targetId = targetIds(l);
        target = QLTarget(opt);

        %project target onto cameras image plane
        %target_I = cam.projectWorldPts(target.pts_W);

        if showPlot 
            %plot
            plotter = QLPlotter(cam, target);
        end
        %flags if correspondence among points will be mixed
        mixPts = false;
        H_C_W_est = cam.estimatePose(target, algoTypes(l), sigmaNoise, mixPts);

        %generate random numbers between x = -2 .. 2 and y = -1 .. 1 for camera
        %position
        t_orig = H_C_W(1:3, 4);
        t_est = H_C_W_est(1:3, 4);

        x_y_error(l,k) = sqrt((t_orig(1)-t_est(1))^2 * (t_orig(2)-t_est(2))^2);
        z_error(l,k) = abs(t_orig(3)-t_est(3));
        
        if showPlot 
            pause(1)
        end
    end
end

%percentiles in x-y - direction
perc100_xy = prctile(x_y_error',100);
perc75_xy = prctile(x_y_error',75);
perc50_xy = prctile(x_y_error',50);
perc25_xy = prctile(x_y_error',25);
perc0_xy = prctile(x_y_error',0);
%percentiles in z - direction
perc100_z = prctile(z_error',100);
perc75_z = prctile(z_error',75);
perc50_z = prctile(z_error',50);
perc25_z = prctile(z_error',25);
perc0_z = prctile(z_error',0);

subplot(1,2,1);
boxplot(x_y_error', {'Alternative' 'RPP'})
title('Positionsfehler in x/y-Richtung')
ylabel('Positionierfehler (m)')
subplot(1,2,2);
boxplot(z_error', {'Alternative' 'RPP'})
title('Positionsfehler in z-Richtung')
ylabel('Positionierfehler (m)')
    
%mean_x_y_error = x_y_error / N
%mean_z_error = z_error / N

% %error calculation
% Hdiff = H_C_W * inv(H_C_W_est);     % Transformation error 
% Rdiff = Hdiff(1:3,1:3);    % Rotation between our answer 
% %and ground truth 
% ang = acos( (trace(Rdiff)-1)/2 ); 
% fprintf('Rotation error (degrees): %f\n', ang*180/pi); 
% tdiff = Hdiff(1:3, 4); 
% fprintf('Translation error: %f\n', norm(tdiff)); 

% %animation
% %definition of start- and endtransformation
% H_C_W_start = qlHomTrans(1, 2, 4, 180*pi/180, 0, 0); % camera position wrt world frame
% H_C_W_end = qlHomTrans(3, 4, 4, 180*pi/180, 0, 0); % camera position wrt world frame
% 
% k = 0;
% dk = 0.1;
% while k < 1
%     %interpolation between start- and endtransformation
%     H_C_W = trinterp(H_C_W_start, H_C_W_end, k);
%     %set camera position in wcs
%     cam.setH_C_W( H_C_W );
%     cam.estimatePose(target, 'RPP', sigmaNoise, false);
%     %wait a second
%     pause(1)
%     %save statistics: runtime, 
%     
%     k = k + dk;
% end









