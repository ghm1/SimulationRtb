%camera class
classdef QLPerspectiveCamera < handle
    
    properties (GetAccess = public, SetAccess = private)
        name  % camera name
        fx     % focal length
        fy     % focal length
        rho   % pixel dimensions in m/pixel
        sensorSize % number of pixel 1x2
        pp    % principal point 1x2
        H_C_W % camera pose wrt world frame
        H_W_C % wcs wrt ccs
        K     % camera Matrix
        Mext  % camera extrinsic
        H_C_BFNED
        
        hg %graphics handle
        
        poseEst        %pose estimation object
        H_C_W_est      %estimated position of camera in WCS
        pts_I_noisy    %noisy projection from position estimation (stored for plotter callback)
        pts_I_rep %reprojected noisy image points
        
        %euler angles with respect to BFNED
        roll
        pitch
        yaw
        
        %camera calibration parameter
        k1
        k2
        p1
        p2
        k3
        
        %undistortion polynom koefficients
        p
        
    end %properties
    
    properties (Access = private)
    	poseEstMethods %pose estimation methods
    end %private properties
    
    methods
        %constructor
        function obj = QLPerspectiveCamera()
            %init default values
        	obj.name = 'cam';    % camera name
%             obj.f    = 0.015;    % focal length
%             obj.sensorSize = [1280 1024];  % number of pixel 1x2
%             obj.pp = obj.sensorSize/2;      % principal point 1x2
%             obj.rho = 10e-6;
            
            obj.fx    = 323.70584;    % focal length (fx/rho)
            obj.fy    = 324.26201;
            obj.sensorSize = [158.5*2 98.5*2];  % number of pixel 1x2
            obj.pp = [160.44416 100.75304];      % principal point 1x2
            obj.rho = 1;
            
            fn = [obj.fx/obj.rho obj.fy/obj.rho];
            
            obj.K = [ fn(1)  0   obj.pp(1);
                       0   fn(2)  obj.pp(2);
                       0   0   1  ];
                   
            %distortion parameter
            obj.k1 = -0.43772;
            obj.k2 = 0.25627;
            obj.p1 = -0.00005;
            obj.p2 = -0.00102;
            obj.k3 = 0.0;
            
            %undistortion polynom coefficients
            obj.findUndistortionPolynomCoefficients();
                   
            obj.roll = 0; obj.pitch = 0; obj.yaw = 0;
                   
            %init default position
            obj.setH_C_W(homogeniousTransform([0; 0; 0], 0, 0, 0));
            %define poseestimation method names for method call
            obj.poseEstMethods = {'DLT', 'DLT_GN', 'EPnP', 'EPnP_GN','RPP' ,'Std'};
            %instantiate pose estimation object
            obj.poseEst = QLPoseEstimation();
        end %function QLPerspectiveCamera
        
        function obj = findUndistortionPolynomCoefficients( obj )
            %For simplicity we only calculate on the x-axis where y equals
            %zero. The values and datarange have to be authentic. The approximation only
            %works on the chosen datarange
            x = 1:obj.sensorSize(1);    %image width
            cc = obj.pp(1);             %principal point
            %we take mean value of normalized focal length (there is not
            %more we can do)
            fc = ( obj.fx + obj.fy ) / 2;
            r_dist = zeros(size(x)); % r distorted
            r_n = zeros(size(x));    % r normalized coordinates

            for m=1 : length(x)
                %normalized image coordinates (undistorted)
                r_n(m) = (x(m)-cc) / fc;
                %calculate r_dist = f(r_n): this adds distortion to normalized image
                %coordinates
                %nomalized image coordinates distorted (for simplicity we only use k1 and k2, this results in an minimal error)
                r_dist(m) = r_n(m) + obj.k1*r_n(m)^3 + obj.k2*r_n(m)^5;
            end

%             %functionplot
%             plot(r_n,r_dist);
%             hold on 
%             %plot angle bisector
%             plot(r_n,r_n, ':k');
%             %plot of inversed function, this is the function we are searching for
%             plot(r_dist,r_n, 'color', 'r');
%             axis equal
%             grid on

            %approximate r_n by polynomial curve fitting:
            %this function maps a distorted value to an undistorted one
            p = polyfit(r_dist,r_n,3);
            r_undist = zeros(size(x)); %undistorted radius
            %interpolate x by polynomia approximation
            %attention: this formular only works on the datarange, that it has been
            %approximated on!
            for m=1 : length(x)
                r_undist(m) = p(1)*r_dist(m)^3 + p(2)*r_dist(m)^2 + p(3)*r_dist(m) + p(4);
            end
            
            %set object member
            obj.p = p;
%             %we are getting a straight line which fits the angle bisector, this is what
%             %we wanted
%             plot(r_n, r_undist, 'color', 'g');

            
        end %findUndistortionPolynomCoefficients
        
        function H_C_W = estimatePose( obj, target, method, noiseSigma, mixPts )
            
            %project target onto img plane
            pts_I = obj.projectWorldPts( target.pts_W );
            
            %punkte zufällig durchmischen
            if(mixPts == true)
                pts_I = obj.doMixPts(pts_I);
            end
            
            %add some noise
            N = length( pts_I(1,:));
            pts_I(1:2,:) = pts_I(1:2,:) + noiseSigma*randn(2,N); 
            %'DLT'
            if strcmp( method, char(obj.poseEstMethods(1)))      
                obj.H_C_W_est = obj.poseEst.estPoseDLT(obj.K, target.pts_W, pts_I );
            %'DLT_GN'
            elseif strcmp( method, char(obj.poseEstMethods(2)))  
                H_C_W_dlt  = obj.poseEst.estPoseDLT(obj.K, target.pts_W, pts_I );
                obj.H_C_W_est = obj.poseEst.estPoseGN( obj.K, target.pts_W, pts_I, H_C_W_dlt );
            %'EPnP'    
            elseif strcmp( method, char(obj.poseEstMethods(3)))
                obj.H_C_W_est = obj.poseEst.estPoseEPnP(obj.K, target.pts_W, pts_I );
            %'EPnP_GN'
            elseif strcmp(method, char(obj.poseEstMethods(4)))
                %obj.H_C_W_est = obj.poseEst.estPoseEPnP_GN(obj.K, target.pts_W, pts_I );
                H_C_W_epnp = obj.poseEst.estPoseEPnP(obj.K, target.pts_W, pts_I );
                obj.H_C_W_est = obj.poseEst.estPoseGN( obj.K, target.pts_W, pts_I, H_C_W_epnp );
            %'RPP'
            elseif strcmp( method, char(obj.poseEstMethods(5)))
                obj.H_C_W_est = obj.poseEst.estPoseRPP(obj.K, target.pts_W, pts_I );
            %'Std'
            elseif strcmp( method, char(obj.poseEstMethods(6)))
               obj.H_C_W_est = obj.poseEst.estPoseStd(obj.K, target.pts_W, pts_I, obj.roll, obj.pitch, obj.yaw, obj.p );
            else
               %unknown method
               disp('unknown method');
               return;
            end
            
            %calculate reprojected image points with estimated
            %transformation
            H_W_C_est = inv(obj.H_C_W_est);
            Mext_est = H_W_C_est( 1:3, : );
            target_I_rep = obj.K * Mext_est * target.pts_W;
            target_I_rep(1,:) = target_I_rep(1,:) ./ target_I_rep(3,:);
            target_I_rep(2,:) = target_I_rep(2,:) ./ target_I_rep(3,:);
            target_I_rep(3,:) = target_I_rep(3,:) ./ target_I_rep(3,:);
            obj.pts_I_rep = target_I_rep;
            
            H_C_W = obj.H_C_W_est;
            obj.pts_I_noisy = pts_I;
            %inform plotter to plot
            notify(obj, 'PoseEstimation');
        end
        
        %setter H_C_W
        function obj = setH_C_W(obj, new_H_C_W)
            obj.H_C_W = new_H_C_W;
            obj.H_W_C = inv(obj.H_C_W);
            obj.Mext = obj.H_W_C(1:3, :);
            
            notify(obj, 'PositionChanged');
        end %function setPositionWCS(H_C_W)
        
        %calculate projection
        function ptsOnImg = projectWorldPts(obj, pts )
            %extrinsic transformation
            ptsOnImg = obj.Mext * pts;

            %normalize homogenious image coordinates (focal length = 1)
            ptsOnImg(1,:) = ptsOnImg(1,:)./ptsOnImg(3,:);
            ptsOnImg(2,:) = ptsOnImg(2,:)./ptsOnImg(3,:);
            ptsOnImg(3,:) = ptsOnImg(3,:)./ptsOnImg(3,:);
            
            %add lens distortion
            for m=1 : length(ptsOnImg)
                %transformat
                r = sqrt( ptsOnImg(1,m)^2 + ptsOnImg(2,m)^2 );
                x = ptsOnImg(1,m);
                y = ptsOnImg(2,m);
                term = 1 + obj.k1*r^2 + obj.k2*r^4 + obj.k3*r^6;
                xstar = x * term + 2 * obj.p1 * x * y + obj.p2 * (r^2 + 2 * x^2);
                ystar = y * term + 2 * obj.p2 * x * y + obj.p1 * (r^2 + 2 * y^2);
                ptsOnImg(1,m) = xstar;
                ptsOnImg(2,m) = ystar;
            end
            
            %intrinsic transformation
            ptsOnImg = obj.K * ptsOnImg;

        end %function projectWorldPts
        
        function obj = setGraphicsHandle(obj, hg)
            obj.hg = hg;
        end % function setGraphicsHandle
        
        function mixedPts = doMixPts(~, pts)
            [~,N] = size(pts);                  
           
            %exchange S times positions of two pts
            for k=1 : N
                %generate two random numbers between 1 and S
                r = randi(N,1,2);
                m = pts(:,r(1));
                pts(:,r(1)) = pts(:,r(2));
                pts(:,r(2)) = m;
            end
            
            mixedPts = pts;
        end %end function mixPts
        
        function obj = setRPY(obj, roll, pitch, yaw)
            obj.roll = roll;
            obj.pitch = pitch;
            obj.yaw = yaw;
        end
        
    end %methods
    
    events
        PositionChanged
        PoseEstimation
    end
    
end %class