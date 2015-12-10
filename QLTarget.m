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
        function obj = QLTarget()
            %initialize properties
            ptDist = 1; % in m
            P1 = [0; 0; 0; 1]; P2 = [ptDist; 0; 0; 1]; P3 = [0; ptDist; 0; 1]; P4 = [-ptDist; 0; 0; 1];
            obj.pts_T = [P1 P2 P3 P4];
            obj.name = 'target';
            
            obj.setH_T_W( qlHomTrans(0, 0, 0, 0, 0, 0));
        end %function QLTarget
        
        function obj = setH_T_W(obj, H_T_W)
            obj.H_T_W = H_T_W;
            obj.pts_W = H_T_W *obj.pts_T;
            
            notify(obj, 'PositionChanged');
        end %function setH_T_W
        
        function obj = setGraphicsHandle(obj, hg)
            obj.hg = hg;
        end % function setGraphicsHandle
    end %methods
    
    events
        PositionChanged
    end
    
end %class