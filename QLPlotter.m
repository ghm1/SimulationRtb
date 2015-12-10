%class that incapsulates plot methods
classdef QLPlotter < handle
    
    properties (GetAccess = public, SetAccess = private)
        name
        imgHandle %handle for image plane
        cam
        target
    end %properties
    
    %public methods
    methods (Access = public)
        %constructor
        function obj = QLPlotter( camera, target )
            obj.name = 'Plotter';
            obj.plotCamera(camera);
            obj.plotTarget(target);
            obj.plotCameraPlane(camera);
            obj.projectTargetPtsOntoCameraPlane(camera, target);
            %addlistener
            addlistener(camera, 'PositionChanged', @(src,evnt)updateCameraPosition(obj,src,evnt));
            addlistener(target, 'PositionChanged', @(src,evnt)updateTargetPosition(obj,src,evnt));
            
            xlabel('X'); ylabel('Y'); zlabel('Z');
            grid on
            axis square
        end  
    end %public methods
    
    methods %(Static)
        function updateCameraPosition( obj, src, evtdata )
            set(src.hg, 'Matrix', src.H_C_W);
            obj.projectTargetPtsOntoCameraPlane(obj.cam, obj.target);
        end %function update updateCameraPosition   
        
        function updateTargetPosition( obj, src, evtdata )
            set(src.hg, 'Matrix', src.H_T_W);
            obj.projectTargetPtsOntoCameraPlane(obj.cam, obj.target);
        end %function update updateTargetPosition   
        
    end % end static methods
    
    %private methods
    methods (Access = private)
        function projectTargetPtsOntoCameraPlane(obj, camera, target)
            hold on
            arglist = {'Marker', 'o', 'MarkerFaceColor', 'k', 'LineStyle', 'none'};
            pts = camera.projectWorldPts(target.pts_W);
            pts = pts(1:2,:);
            
            plot(pts(1,:,1), pts(2,:,1), arglist{:}, 'Parent', obj.imgHandle);

            hold off
        end
            
        function plotTarget( obj, target ) 
            if( isa(target,'QLTarget') )
                hold on;
                obj.target = target;
                %plot target frame
                hg = obj.drawTarget(target);
                %set target transformation
                set(hg, 'Matrix', target.H_T_W);
                
                hold off;
            else
                disp('This is a plot function for objects of type QLTarget!');
            end
        end % function plotTarget        
        
        function plotCamera( obj, cam )
            if( isa(cam,'QLPerspectiveCamera') )
                hold on;          
                obj.cam = cam;
                %draw camera housing and frame
                hg = obj.drawCamera(cam);
                %set camera transformation
                set( hg, 'Matrix', cam.H_C_W);
                
                hold off;             
            else
                disp('This is a plot function for objects of type QLPerspectiveCamera!');
            end        
            
        end % function plotCamera
        
        function plotCameraPlane(obj, cam)
            figure
            obj.imgHandle = axes;
            fig = get(obj.imgHandle, 'Parent');
            disp('make axes');
            axis square
            set(fig, 'MenuBar', 'none');
            set(fig, 'Tag', 'camera');
            set(obj.imgHandle, 'Color', [1 1 0.8]);
            set(fig, 'HandleVisibility', 'off');
            set(fig, 'name', [class(cam) ':' cam.name]);
            
            % create an axis for camera view
            limits = [0 cam.sensorSize(1) 0 cam.sensorSize(2)];
            set(obj.imgHandle, 'XLim', limits(1:2), 'YLim', limits(3:4), ...
                'DataAspectRatio', [1 1 1], ...
                'Xgrid', 'on', 'Ygrid', 'on', ...
                'Ydir' , 'reverse', ...
                'NextPlot', 'add', ...
                'Tag', obj.name ...
            );
        
            xlabel(obj.imgHandle, 'u (pixels)');
            ylabel(obj.imgHandle, 'v (pixels)');
            figure( fig );   % raise the camera view
            set(obj.imgHandle, 'NextPlot', 'replacechildren');
        end
        
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
            target.setGraphicsHandle(hg);
            
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

            opt.color = 'r';
            opt.mode = 'solid';
            opt.label = false;
            opt.scale = 1/3;        
            s = opt.scale;

            % create a new transform group
            hg = hgtransform;
            cam.setGraphicsHandle(hg);

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

                %s = opt.scale;
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