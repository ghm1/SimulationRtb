%camera class
classdef QLPerspectiveCamera < handle
    
    properties (GetAccess = public, SetAccess = private)
        name  % camera name
        f     % focal length
        rho   % pixel dimensions in m/pixel
        sensorSize % number of pixel 1x2
        pp    % principal point 1x2
        H_C_W % camera pose wrt world frame
        H_W_C % wcs wrt ccs
        K     % camera Matrix
        Mext  % camera extrinsic
        
        hg %graphics handle
        
        poseEst        %pose estimation object
        H_C_W_est      %estimated position of camera in WCS
        pts_I_noisy    %noisy projection from position estimation (stored for plotter callback)
        pts_I_rep %reprojected noisy image points
    end %properties
    
    properties (Access = private)
    	poseEstMethods %pose estimation methods
    end %private properties
    
    methods
        %constructor
        function obj = QLPerspectiveCamera()
            %init default values
        	obj.name = 'cam';    % camera name
            obj.f    = 0.015;    % focal length
            obj.sensorSize = [1280 1024];  % number of pixel 1x2
            obj.pp = obj.sensorSize/2;      % principal point 1x2
            obj.rho = 10e-6;
            
            fn = obj.f/obj.rho;
            obj.K = [ fn  0   obj.pp(1);
                       0   fn  obj.pp(2);
                       0   0   1  ];
                   
            %init default position
            obj.setH_C_W(homogeniousTransform([0; 0; 0], 0, 0, 0));
            %define poseestimation method names for method call
            obj.poseEstMethods = {'DLT', 'DLT_GN', 'EPnP', 'EPnP_GN', 'Std'};
            %instantiate pose estimation object
            obj.poseEst = QLPoseEstimation();
        end %function QLPerspectiveCamera
        
        function H_C_W = estimatePose( obj, target, method, noiseSigma )
            
            %project target onto img plane
            pts_I = obj.projectWorldPts( target.pts_W );
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
                obj.H_C_W_est = obj.poseEst.estPoseEPnP_GN(obj.K, target.pts_W, pts_I );
            %'Std'
            elseif strcmp( method, char(obj.poseEstMethods(5)))
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
            ptsOnImg = obj.K * obj.Mext * pts;
            ptsOnImg(1,:) = ptsOnImg(1,:)./ptsOnImg(3,:);
            ptsOnImg(2,:) = ptsOnImg(2,:)./ptsOnImg(3,:);
            ptsOnImg(3,:) = ptsOnImg(3,:)./ptsOnImg(3,:);
        end %function projectWorldPts
        
        function obj = setGraphicsHandle(obj, hg)
            obj.hg = hg;
        end % function setGraphicsHandle
        
    end %methods
    
    events
        PositionChanged
        PoseEstimation
    end
    
end %class