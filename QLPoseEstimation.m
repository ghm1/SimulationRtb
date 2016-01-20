%Position estimation

classdef QLPoseEstimation
    
    properties  (GetAccess = public, SetAccess = private)
    end %end properties
    
    methods
        %constructor
        function obj = QLPoseEstimation(obj)
        end %QLPoseEstimation
        
        function H_C_W = estPoseStd(obj, K, pts_W, pts_I, roll, pitch, yaw, p, distorted )
            %In this case we assume, that we know r,p,y and the
            %transformation of the target with respect to the world frame (H_T_W) 
            
            %add noise to euler angles
            noiseSigma = 0.02;
            roll_n = roll + noiseSigma*randn(1,1);
            pitch_n = pitch + noiseSigma*randn(1,1);
            yaw_n = yaw + noiseSigma*randn(1,1);
            
            R_C_BFNED = rpy2r(roll_n, pitch_n, yaw_n);
            %q = Quaternion(R_C_BFNED);
            %R_C1_C2 = R_C2_C1; %as the rotationmatix is orthonormal, the transposed is the same as the inverse
            %pts_I2 = K * R_C_BFNED / K * pts_I; % R_C1_C2 / K  means R_C1_C2 * inv(K)
            %pts_I2 = [ pts_I2(1,:) ./ pts_I2(3,:); pts_I2(2,:) ./ pts_I2(3,:) ];
                
            %we first normalize the image coordinates and then rotate the
            %resulting normalized coordinates into an virtual orthogonal
            %camera
            %intrinsic transform
            pts_I2 = inv(K) * pts_I; % R_C1_C2 / K  means R_C1_C2 * inv(K)
            %normalization
            %pts_I2 = [ pts_I2(1,:) ./ pts_I2(3,:); pts_I2(2,:) ./ pts_I2(3,:); pts_I2(3,:) ./ pts_I2(3,:) ];
            
            if distorted
                %remove distortion
                for m=1 : length(pts_I2)
                    x = pts_I2(1,m);
                    y = pts_I2(2,m);
                    %transform to polar coordinates
                    phi = atan2(y,x);
                    r_dist = sqrt(x^2+y^2);
                    r_undist = p(1)*r_dist^3 + p(2)*r_dist^2 + p(3)*r_dist + p(4);
                    x_undist = r_undist * cos(phi);
                    y_undist = r_undist * sin(phi);
                    pts_I2(1,m) = x_undist;
                    pts_I2(2,m) = y_undist;
                end
            end

            %normalization
            pts_I2 = [ pts_I2(1,:) ./ pts_I2(3,:); pts_I2(2,:) ./ pts_I2(3,:); pts_I2(3,:) ./ pts_I2(3,:) ];  
            %homography
            pts_I2 = R_C_BFNED * pts_I2;
            %again normalization
            pts_I2 = [ pts_I2(1,:) ./ pts_I2(3,:); pts_I2(2,:) ./ pts_I2(3,:); pts_I2(3,:) ./ pts_I2(3,:) ];  
    
            %sort points
            [L,R,M,F] = obj.sortPts( pts_I2 );
            
%             %distance LR
%             realDist = obj.eDist(pts_W(1:2,2),pts_W(1:2,4));           
%             imgDistM = obj.eDist(L,R);
%             %evluate height
%             height = realDist ./ imgDistM;
%             
            %mean distance LR, LF, RF
            realDistMean = obj.eDist(pts_W(1:2,2),pts_W(1:2,4)) + 2 * obj.eDist(pts_W(1:2,2),pts_W(1:2,3));
            imgDistMean = obj.eDist(L,R) + obj.eDist(L,F) + obj.eDist(F,R);
            %evluate height mean
            height = realDistMean ./ imgDistMean;

            % M is equal to the shift onto target frame center 
            pose_xy = M * height;
            %translation of target wrt BFNED (target was transformed to BFNED)
            t_T_BFNED = [pose_xy(1); pose_xy(2); height];       
            t_T_BFNED = [t_T_BFNED; 1];
            %world to LNED Transformation
            H_W_LNED = [ 0 1 0 0;
                         1 0 0 0;
                         0 0 -1 0;
                         0 0 0 1 ];
            %position of target frame in world frame is just the M-point in
            %the target-world pointset.
            t_T_W = pts_W(1:3,1);
            t_T_W = [t_T_W; 1];
            %translation of target wrt LNED
            t_T_LNED = H_W_LNED * t_T_W;
            
            %Position of BFNED wrt. LNED (we can do this because BFNED and LNED are parallel)
            t_BFNED_LNED = t_T_LNED - t_T_BFNED;
            t_BFNED_LNED(4) = 1;
            
            H_C_LNED = [[ R_C_BFNED; zeros(1,3)] [t_BFNED_LNED]];
            H_C_W = inv(H_W_LNED) * H_C_LNED;            
        end
        
        function [L,R,M,F] = sortPts( obj, pts_I2 )
            %1. calc distances
            keySet = [[1 2]; [1 3]; [1 4]; [2 3]; [2 4]; [3 4] ];
            
            valueSet = [ obj.eSqrdDist( pts_I2(:,1), pts_I2(:,2)),
                         obj.eSqrdDist( pts_I2(:,1), pts_I2(:,3)),
                         obj.eSqrdDist( pts_I2(:,1), pts_I2(:,4)),
                         obj.eSqrdDist( pts_I2(:,2), pts_I2(:,3)),
                         obj.eSqrdDist( pts_I2(:,2), pts_I2(:,4)),
                         obj.eSqrdDist( pts_I2(:,3), pts_I2(:,4)),
                       ];
             
             maxI = 1;
             N = length(valueSet);
             for k=1 : N
                 for m=k : N
                    if valueSet(k) >  valueSet(maxI)
                        maxI = k;
                    end
                 end
             end
             
             %extract points 
             id = keySet(maxI,:);
             LR = [ pts_I2(:,id(1)) pts_I2(:,id(2)) ];
             pts_I2(:,id(1)) = [];
             pts_I2(:,id(2)-1) = [];
             
             %find point with smaller distance to line LR
             lineLR = LR(:,2) - LR(:,1);
             %normalize
             lineLR = lineLR ./ sqrt( lineLR(1)^2 + lineLR(2)^2 );
             
             lineP1 = pts_I2(:,1) - LR(:,1);
             projPt1 = LR(:,1) + dot(lineLR, lineP1) * lineLR;
             dist1 = obj.eDist(projPt1, pts_I2(:,1));
             
             lineP2 = pts_I2(:,2) - LR(:,1);
             projPt2 = LR(:,1) + dot(lineLR, lineP2) * lineLR;
             dist2 = obj.eDist(projPt2, pts_I2(:,2));
             
             if (dist1 > dist2)
                 F = pts_I2(:,1);
                 M = pts_I2(:,2);
             else
                 F = pts_I2(:,2);
                 M = pts_I2(:,1); 
             end
             
             %sort LR to L and R
             res = (LR(1,1)-M(1,1))*(F(2,1)-M(2,1)) - (LR(2,1)-M(2,1))*(F(1,1)-M(1,1));              
             if res > 0
                L = LR(:,1);
                R = LR(:,2);
             else
                R = LR(:,1);
                L = LR(:,2);
             end
        end
        
        function dist = eDist(~, P1, P2)
            dist = P1 - P2;
            dist = sqrt(dist(1)^2 + dist(2)^2);
        end
        
        function dist = eSqrdDist(~, P1, P2)
            dist = P1 - P2;
            dist = dist(1)^2 + dist(2)^2;
        end
        
        function H_C_W = estPoseRPP(obj, K, pts_W, pts_I)  
            %algorithm for planar targets
            pts_C = inv(K)*pts_I;
            target_W = pts_W(1:3,:);
            %model,iprts,opt
            [pose,po2] = rpp(target_W, pts_C);
            
            H_W_C = [pose.R pose.t; [zeros(1,3) 1]];
            H_C_W = inv( H_W_C );
        end %function estPoseRPP
        
        function H_C_W = estPoseEPnP_GN(obj, K, pts_W, pts_I)
            [R, t] = efficient_pnp_gauss(pts_W', pts_I', K);

            H_C_W = inv([R t; 0 0 0 1]);
        end %function estPoseEPnP   
        
        function H_C_W = estPoseEPnP(obj, K, pts_W, pts_I)
            [R, t] = efficient_pnp(pts_W', pts_I', K);
            %R and t are R_W_C and t_W_C
            H_C_W = inv([R t; 0 0 0 1]);
        end %function estPoseEPnP       
        
        %position optimization with gauss-newton method
        function H_C_W = estPoseGN(obj, K, pts_W, pts_I, H_C_W_guess)
            
            %transform H_C_W_guess in x
            H_W_C_guess = inv(H_C_W_guess);
            t_W_C = H_W_C_guess(1:3,4);
            rpy = tr2rpy(H_W_C_guess);
            %required [ax; ay; az; tx; ty; tz] 
            x = [rpy(1); rpy(2); rpy(3); t_W_C];            
            
            xOpt = poseEstimationNewton(x, pts_I(1:2,:), pts_W, K);
            
            %transform xOpt to H_C_W
            H_W_C = qlHomTrans(xOpt(4), xOpt(5), xOpt(6), xOpt(1), xOpt(2), xOpt(3)); 
            H_C_W = inv(H_W_C);
        end %function estPoseGN
        
        function H_C_W = estPoseDLT(obj, K, pts_W, pts_I)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Solve for the pose of the model with respect to the camera.
            [m,N] = size(pts_W);
            pts_C = inv(K)*pts_I;  % Normalize image points
            
            % Ok, now we have pn = Mext*pts_W.
            % If we know pts_W and pn, we can solve for the elements of Mext.
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
                X = pts_W(1,i);   Y = pts_W(2,i);   Z = pts_W(3,i);
                x = pts_C(1,i);    y = pts_C(2,i);
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
            
            H_C_W = Hestimated_c_w;
        end %function estPoseDLT      
        
    end %end methods
    
end %class QLPoseEstimation
