%class that incapsulates plot methods
classdef QLPlotter
    
    properties (GetAccess = public, SetAccess = private)
        worldFrame  % dimensions of world frame drawing object
        box % box that surrounds the whole scene
    end %properties
    
    %public methods
    methods (Access = public)
        %constructor
        function obj = QLPlotter()
            %definition of world frame
            worldOrigin = [0; 0; 0];
            worldX = [1; 0; 0];
            worldY = [0; 1; 0];
            worldZ = [0; 0; 1];
            %homogenous world frame
            obj.worldFrame = [worldOrigin worldX worldY worldZ; ones(1,4)];
            
            %update box
            obj.box = obj.worldFrame * 2;
            
            %plot world frame
            %obj.plotFrame( qlHomTrans(0, 0, 0, 0, 0, 0), 'W');
            
            xlabel('X'); ylabel('Y'); zlabel('Z');
            grid on
        end
        
        
        function plotCamera( obj, cam )
            if( isa(cam,'QLPerspectiveCamera') )
                hold on;
               
                %draw camera housing and frame
                hg = obj.drawCamera(cam);
                %set camera transformation
                set(hg, 'Matrix', cam.H_C_W);
                
                hold off;             
            else
                disp('This is a plot function for objects of type QLPerspectiveCamera!');
            end        
            
        end % function plotCamera
        
        function plotTarget( obj, target ) 
            if( isa(target,'QLTarget') )
                hold on;
                %plot target frame
                hg = obj.drawTarget(target);
                %set target transformation
                set(hg, 'Matrix', target.H_T_W);
                
                hold off;
            else
                disp('This is a plot function for objects of type QLTarget!');
            end
        end % function plotTarget
        
    end %public methods
    
    %private methods
    methods (Access = private)
        
        function hg = drawTarget(obj, target)
            
            opt.color = 'b';
            opt.mode = 'solid';
            opt.label = true;
            opt.scale = 1/3;
            opt.size = 30;
            s = opt.scale;
            a = 3;          % length of axis line segments
            
            % create a new transform group
            hg = hgtransform;
            
            %plot target points
            pts = target.pts_T;
            for k=1:length(pts)
                plot3(pts(1,k), pts(2,k), pts(3,k), 'Marker', '.', 'MarkerSize', opt.size, 'Color', opt.color, 'Parent', hg);
            end            
            
            % draw the x-, y- and z-axes
            plot3([0,a*s], [0,0], [0,0], 'k', 'Parent', hg);
            text(a*s, 0, 0, sprintf(' X'), 'Parent', hg);
            plot3([0,0], [0,a*s], [0,0], 'k', 'Parent', hg);
            text(0, a*s, 0, sprintf(' Y'), 'Parent', hg);
            plot3([0,0], [0,0], [0,a*s], 'k', 'Parent', hg);
            text(0, 0, a*s, sprintf(' Z'), 'Parent', hg);
            
            if opt.label
                text( 0.3*a*s, 0.1*a*s, 0, target.name, 'Parent', hg);
            end
            
        end %function drawTarget
        
        function hg = drawCamera(obj, cam)

            opt.color = 'b';
            opt.mode = 'solid';
            opt.label = false;
            opt.scale = 1/3;        
            s = opt.scale;

            % create a new transform group
            hg = hgtransform;

            % the box is centred at the origin and its centerline parallel to the
            % z-axis.  Its z-extent is -bh/2 to bh/2.
            bw = 0.5;       % half width of the box
            bh = 1.2;       % height of the box
            cr = 0.4;       % cylinder radius
            ch = 0.8;       % cylinder height
            cn = 16;        % number of facets of cylinder
            a = 3;          % length of axis line segments

            opt.parent = hg;

            % draw the box part of the camera
            r = bw*[1; 1];
            x = r * [1 1 -1 -1 1];
            y = r * [1 -1 -1 1 1];
            z = [-bh; bh]/2 * ones(1,5);
            draw(x,y,z, opt);

            % draw top and bottom of box
            x = bw * [-1 1; -1 1];
            y = bw * [1 1; -1 -1];
            z = [1 1; 1 1];

            draw(x,y,-bh/2*z, opt);
            draw(x,y,bh/2*z, opt);


            % draw the lens
            [x,y,z] = cylinder(cr, cn);
            z = ch*z+bh/2;
            h = draw(x,y,z, opt);
            set(h, 'BackFaceLighting', 'unlit');

            % draw the x-, y- and z-axes
            plot3([0,a*s], [0,0], [0,0], 'k', 'Parent', hg);
            text(a*s, 0, 0, sprintf(' X'), 'Parent', hg);
            plot3([0,0], [0,a*s], [0,0], 'k', 'Parent', hg);
            text(0, a*s, 0, sprintf(' Y'), 'Parent', hg);
            plot3([0,0], [0,0], [0,a*s], 'k', 'Parent', hg);
            text(0, 0, a*s, sprintf(' Z'), 'Parent', hg);

            if opt.label
                text( 0.3*a*s, 0.1*a*s, 0, cam.name, 'Parent', hg);
            end

            function h = draw(x, y, z, opt)

                s = opt.scale;
                switch opt.mode
                case 'solid'
                    h = surf(x*s,y*s,z*s, 'FaceColor', opt.color);
                case 'surfl'
                    h = surfl(x*s,y*s,z*s, 'FaceColor', opt.color);
                case 'mesh'
                    h = mesh(x*s,y*s,z*s, 'EdgeColor', opt.color);
                end

                set(h, 'Parent', opt.parent);
            end
        end
        
    end %methods
end %class