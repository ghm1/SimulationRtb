%class that incapsulates plot methods
classdef QLPlotter
    
    properties (GetAccess = public, SetAccess = private)
        worldFrame
    end %properties
    
    methods
        %constructor
        function obj = QLPlotter()
            %definition of world frame
            worldOrigin = [0; 0; 0];
            worldX = [1; 0; 0];
            worldY = [0; 1; 0];
            worldZ = [0; 0; 1];
            obj.worldFrame = [worldOrigin worldX worldY worldZ];
            
            %plot world frame
            obj.plotFrame( qlHomTrans(0, 0, 0, 0, 0, 0));
            hold on
        end
        
        function plotFrame( H )
            %copy and transform
            
            %correct axis
        end
    end %methods
end %class