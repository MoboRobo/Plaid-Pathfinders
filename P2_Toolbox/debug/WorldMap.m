% Creates a Persistent Map of the Robot in its World
classdef WorldMap
    %% PROPERTIES
    properties (GetAccess=public, SetAccess=private)
        robot;       % Robot that Exists in this World
        
        boundary;    % Line Object of World Boundaries
        obstacles;   % Vector of Line Objects defining Obstacles in the World (which robot can collide with)
        flags;       % Vector of Line Objects defining Significant Points in the World (imaginary, robot can't collide with these)
        
        map;         % Line Map of all Objects in the World (boundary, obstacles, flags)
    end % WorldMap->properties(public,private)
    
    methods
        %% Constructor
        % r - Robot this Debugger is Tracking
        % b - Bounds of the World given as a Vector of Points. Example:
        %     [0 0; 0 1; 1 1; 1 0; 0 0] is a 1m. square touching the origin
        function obj = WorldMap(r, b)
            if nargin>1
                if isa(r,'P2_Robot')
                    obj.robot = r;
                else
                    error('Debugger Robot must be a P2_Robot')
                end % r is P2_Robot?
                
                obj.boundary = lineObject();
                obj.boundary.lines = b;
            else
                error('Must give Debugger a Robot to Track and World Bounds')
            end% nargin>=2?
            
            obj.obstacles = [lineObject()];
             obj.obstacles(1).lines = [0 0]; %Must be instantiated with contents
            obj.flags = [lineObject()];
             obj.flags(1).lines = [0 0]; %Must be instantiated with contents
            
            obj.createMap();
        end % #WorldMap Constructor
        
        %% Create Map
        % Creates a New Map (containing updates to boundary, obstacles,
        % flags) and Loads the Robot into it.
        function createMap(obj)
            obj.map = lineMap([obj.boundary obj.obstacles obj.flags]);
            h = obj.map.plot(); %Map Plot
            if ishandle(h)
                close(h);
            end
            f = figure(); % Must create and close a figure around genMap to 
                          % keep objects from being plotted in another
                          % figure.
                obj.robot.core.genMap(obj.map.objects);
            close(f);
        end % #createMap
        
        %% Add Flag
        % As a Non-Colliding Flag to the WorldMap and Updates the Plot
        % f - Point-Outline of the Flag given as a Vector of Points. Ex:
        %     [0 0; 0 1; 1 1; 1 0; 0 0] is a 1m. square touching the origin
        function addFlag(obj, f)
            obj.flags(end + 1) = lineObject();
            obj.flags(end).lines = f;%(nb: new end)
            
            obj.createMap(); %Update display but creating new map, replacing current
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
            
            obj.createMap(); %Update display but creating new map, replacing current
        end % #addFlags
        
        %% Add Obstacle
        % As a Colliding Obstacle to the WorldMap and Updates the Plot
        % o - Point-Outline of the Obstacle given as a Vector of Points. Ex:
        %     [0 0; 0 1; 1 1; 1 0; 0 0] is a 1m. square touching the origin
        function addObstacle(obj, o)
            obj.flags(end + 1) = lineObject();
            obj.flags(end).lines = o;%(nb: new end)
            
            obj.createMap(); %Update display but creating new map, replacing current
        end % #addObstacle
        
        %% Add Obstacles
        % As a Colliding Obstacle to the WorldMap and Updates the Plot
        % os - Vector of Point-Outlines of the Obstacles given as a Vector of Points. Ex:
        %     [0 0; 0 1; 1 1; 1 0; 0 0] is a 1m. square touching the origin
        function addObstacles(obj, os)
            for o = os
                obj.flags(end + 1) = lineObject();
                obj.flags(end).lines = o;%(nb: new end)
            end %f in fs
            
            obj.createMap(); %Update display but creating new map, replacing current
        end % #addObstacles
        
    end % Debugger->methods
end % WorldMap