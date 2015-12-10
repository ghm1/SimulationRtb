%Position estimation

classdef QLPoseEstimation
    
    properties  (GetAccess = public, SetAccess = private)
    end %end properties
    
    methods
        %constructor
        function obj = QLPoseEstimation(obj)
        end %QLPoseEstimation
        
        function H_C_W = estPoseEPnP(obj, K, pts_W, pts_I)
            [R, t] = efficient_pnp(pts_W', pts_I', K);

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
