% Container Class (~namespace) for Generating Lists of Points Defining the Outline of a
% Shape.
% (All closed shapes have the last point being coincident to the first
% point as is required for lineObjects).
classdef ShapeGen
    
    methods(Static)
        
        %% Rectangle
        % Creates a Rectangle of Width, w, and Height, h, Centered at [0 0]
        function pts = rect(w,h)
            pts = [w/2 -h/2; w/2 h/2; -w/2 h/2; -w/2 -h/2; w/2 -h/2];
        end % #rect
        
        %% n-Gon
        % Creates an n-Pointed Polygon Centered at [0 0] with Each Point
        % being a Distance R away from the origin [0 0].
        function pts = ngon(n, R)
            pts  = zeros(n+1,2);
            del = 2*pi/n; % Angular spacing between points.
            
            for(i = 1:n)
                ang = i*del;
                pts(i,:) = [R*cos(ang) R*sin(ang)];
            end % for i = 1:n
            
            pts(end,:) = pts(1,:); % Close shape
        end % #ngon
        
        %% Translate Points
        % Translates a Points List by xx, yy.
        function out = translatePts(in, xx,yy)
            out = zeros(size(in));
            for(i = 1:length(in))
                out(i,:) = [in(i,1)+xx in(i,1)+yy];
            end % for i = 1:len(n)
        end % #translatePts
        
    end % ShapeGen->methods
    
end % ShapeGen