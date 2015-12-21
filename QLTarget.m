%Target class
classdef QLTarget < handle
    
    properties (GetAccess = public, SetAccess = private)
        name
        H_T_W       %position with respect to wcs
        pts_T    %target points in target cs
        pts_W    %target pointpositions in wcs
        
        hg       %graphics handle
    end %properties
    
    methods
        %constructor
        function obj = QLTarget( opt )
            %initialize properties
            
            if opt.randomTarget
            	%random Targets
                if opt.planar
                    target_T = rand(2,opt.nPts)*0.5;
                    target_T = [target_T; zeros(1,opt.nPts); ones(1,opt.nPts)];
                else
                    target_T = rand(3,opt.nPts)*0.5;
                    target_T = [target_T; ones(1,opt.nPts)];                   
                end % opt.planar
                obj.pts_T = target_T;
            else
                %predefined Targets
                ptDist = 1.0; % in m
                if(opt.targetId == 1)
                    P1 = [0; 0; 0; 1]; P2 = [ptDist; 0; 0; 1]; P3 = [0; ptDist; 0; 1]; ...
                        P4 = [-ptDist; 0; 0; 1];
                    obj.pts_T = [P1 P2 P3 P4];
                elseif(opt.targetId == 2)
                    P1 = [-ptDist/2; 0; 0; 1]; P2 = [ptDist/2; 0; 0; 1]; P3 = [ptDist; ptDist; 0; 1]; ...
                        P4 = [-ptDist; ptDist; 0; 1];  P5 = [ptDist; -ptDist; 0; 1]; P6 = [-ptDist; -ptDist; 0; 1];
                    obj.pts_T = [P1 P2 P3 P4 P5 P6];
                elseif(opt.targetId == 3)
                    P1 = [-ptDist/2; 0; ptDist; 1]; P2 = [ptDist/2; 0; ptDist; 1]; P3 = [ptDist; ptDist; 0; 1]; ...
                        P4 = [-ptDist; ptDist; 0; 1];  P5 = [ptDist; -ptDist; 0; 1]; P6 = [-ptDist; -ptDist; 0; 1];
                    obj.pts_T = [P1 P2 P3 P4 P5 P6];        
                elseif(opt.targetId == 4)
                    P1 = [-0.56; 0; 1.0; 1]; P2 = [1.2; 0; 0; 1]; P3 = [5.5; 1.2; 0; 1]; ...
                        P4 = [-6.5; ptDist; 0; 1];  P5 = [3.3; -3.3; 0; 1]; P6 = [-4.2; -2.3; 0; 1];
                    obj.pts_T = [P1 P2 P3 P4 P5 P6];
                elseif(opt.targetId == 5)
                    P1 = [0; 0; 0; 1]; P2 = [ptDist; 0; 0; 1]; P3 = [0; ptDist; 0; 1]; ...
                        P4 = [-ptDist; 0; 0; 1];  P5 = [ptDist; ptDist; 0; 1]; P6 = [-ptDist; ptDist; 0; 1];
                    obj.pts_T = [P1 P2 P3 P4 P5 P6];   
                else
                    disp('target does not exist')
                end
            end %opt.randomTarget
            
            %set name
            obj.name = 'target';          
            %transform TCS-Pts to WCS
            obj.setH_T_W( opt.H_T_W );
                
        end %function QLTarget
        
        function obj = setH_T_W(obj, H_T_W)
            obj.H_T_W = H_T_W;
            obj.pts_W = H_T_W *obj.pts_T;
            
            notify(obj, 'PositionChanged');
        end %function setH_T_W
        
        function obj = setGraphicsHandle(obj, hg)
            obj.hg = hg;
        end % function setGraphicsHandle
        
        function obj = setPts( obj, newPts, H_T_W )
            obj.pts_T = newPts;
            obj.setH_T_W( H_T_W );
        end %function setPts
        
    end %methods
    
    events
        PositionChanged
    end
    
end %class