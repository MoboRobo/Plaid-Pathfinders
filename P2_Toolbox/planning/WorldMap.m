% Creates a Persistent Map of the Robot in its World
classdef WorldMap < handle
    %% PROPERTIES
    properties (GetAccess=public, SetAccess=private)
        boundary;    % Line Object of World Boundaries
        obstacles;   % Vector of Line Objects defining Obstacles in the World (which robot can collide with)
        flags;       % Vector of Line Objects defining Significant Points in the World (imaginary, robot can't collide with these)
        
        map;         % Line Map of all Objects in the World (boundary, obstacles, flags)
    end % WorldMap->properties(public,private)
    
    methods
        %% Constructor
        % b - Bounds of the World given as a Vector of Points. Example:
        %     [0 0; 0 1; 1 1; 1 0; 0 0] is a 1m. square touching the origin
        function obj = WorldMap(b)
            if nargin>=1
                obj.boundary = lineObject();
                obj.boundary.lines = b;
            else
                error('Must give World Bounds')
            end% nargin>=1?
            
            obj.obstacles = [lineObject()];
             obj.obstacles(1).lines = [0 0]; %Must be instantiated with contents
            obj.flags = [lineObject()];
             obj.flags(1).lines = [0 0]; %Must be instantiated with contents
            
            obj.createMap();
        end % #WorldMap Constructor
        
        %% Create Map
        % Creates and returns a New LineMap (containing updates to boundary,
        % obstacles, flags) representing the contents of this WorldMap.
        function map = createMap(obj)
            obj.map = lineMap([obj.boundary obj.obstacles obj.flags]);
            map = obj.map;
        end % #createMap
        
        %% Create Point Vectors
        % Returns two vectors for every line in a colliding object in the 
        % map in the map (this excludes flags but includes the boundary and 
        % obstacles):
        % pts_s - vector of the start points of the included lines
        % pts_e - vector of the end points of the included lines
        function [pts_s, pts_e] = createPointVecs(obj)
            pts_s = [0;0];
            pts_e = [0;0];
            for b = obj.boundary
                bs = b.lines';
                bs_s = bs(:,1:end-1);
                bs_e = bs(:,2:end);
                pts_s = [pts_s bs_s];
                pts_e = [pts_e bs_e];
            end
            for o = obj.obstacles
                os = o.lines';
                os_s = os(:,1:end-1);
                os_e = os(:,2:end);
                
                pts_s = [pts_s os_s];
                pts_e = [pts_e os_e];
            end
        end % #createPointVecs
        
        %% Add Flag
        % As a Non-Colliding Flag to the WorldMap and Updates the Plot
        % f - Point-Outline of the Flag given as a Vector of Points. Ex:
        %     [0 0; 0 1; 1 1; 1 0; 0 0] is a 1m. square touching the origin
        % Returns: the lineObject representing the flag (the pose of this
        % object can be updated to move it across the world).
        function out = addFlag(obj, f)
            obj.flags(end + 1) = lineObject();
            obj.flags(end).lines = f;%(nb: new end)
            out = obj.flags(end);
            
            obj.createMap();
        end % #addFlag
        
        %% Add Flags
        % As Non-Colliding Flags to the WorldMap and Updates the Plot
        % fs - Vector of Point-Outlines of the Flags given as a Vector of Points. Ex:
        %     [0 0; 0 1; 1 1; 1 0; 0 0] is a 1m. square touching the origin
        function addFlags(obj, fs)
            for f = fs
                obj.flags(end + 1) = lineObject();
                obj.flags(end).lines = f;%(nb: new end)
            end %f in fs
            
            obj.createMap();
        end % #addFlags
        
        %% Add Obstacle
        % As a Colliding Obstacle to the WorldMap and Updates the Plot
        % o - Point-Outline of the Obstacle given as a Vector of Points. Ex:
        %     [0 0; 0 1; 1 1; 1 0; 0 0] is a 1m. square touching the origin
        % Returns: the lineObject representing the flag (the pose of this
        % object can be updated to move it across the world).
        function out = addObstacle(obj, o)
            obj.obstacles(end + 1) = lineObject();
            obj.obstacles(end).lines = o;%(nb: new end)
            out = obj.obstacles(end);
            
            obj.createMap();
        end % #addObstacle
        
        %% Add Obstacles
        % As a Colliding Obstacle to the WorldMap and Updates the Plot
        % os - Vector of Point-Outlines of the Obstacles given as a Vector of Points. Ex:
        %     [0 0; 0 1; 1 1; 1 0; 0 0] is a 1m. square touching the origin
        function addObstacles(obj, os)
            for o = os
                obj.obstacles(end + 1) = lineObject();
                obj.obstacles(end).lines = o;%(nb: new end)
            end %f in fs
            
            obj.createMap();
        end % #addObstacles
        
        %% Plot
        % Plots the Given World Map. Optionally, specify the axis to plot
        % into (otherwise, uses gca). Returns Vector of Handles.
        function h = plot(obj, aa)
            if nargin>1
                as = aa;
            else
                as = gca;
            end
            
            h = plot(as, obj.boundary.lines(:,2), obj.boundary.lines(:,1), 'b');
            
            for o = obj.obstacles
                h(end+1) = plot(as, o.lines(:,2), o.lines(:,1), 'b');
            end
            for f = obj.flags
                h(end+1) = plot(as, f.lines(:,2), f.lines(:,1), 'm');
            end
        end % #plot
        
    end % WorldMap ->methods
end % WorldMap