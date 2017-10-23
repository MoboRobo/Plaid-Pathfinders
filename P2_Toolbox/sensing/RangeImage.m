%  Class for Storing and Manipulating Structured Light Range Finder
% (~Lidar) Data.
%
% A note on coordinate system: +ve x points forward out of robot, +ve y
% points to the left, th (bearing) ranges from 0 at +ve x axis, going CCW
% to pi at -ve x axis and CW to -pi.
%                     +x
%                     |
%                     |
%                   0 | 0
%           +y -------+-------
%                     |
%                     |
%                 +pi | -pi
classdef RangeImage < handle
    %% STATIC & CONSTANT PROPERTIES
    properties(Constant)
        MIN_RANGE = 0.07;   % m, Minimum Range for Sensible Data
        MAX_RANGE = 1.25;   % m, Maximum Range for Sensible Data
    end
    % Must be handled as persistent variables wrapped in an accessor which
    % returns the value and sets the value if given an argument.
    methods (Static)
        % [deg] Offset of index 0 in the range data to 0 bearing on the 
        % robot (+ve offset -> index 0 starts on robot's left side).
        function val = INDEX_OFFSET(new_value)
            persistent v
            if isempty(v)
                v = 5; %Default value
            end
            if nargin %Set value?
                v = new_value;
            end
            val = v;
        end
        
        % [deg] Angular Size of Each Laser Index. Ex: If i=0 is at 0 deg,
        % i=1 is at INDEX_SIZE() degrees, i=2 at 2*INDEX_SIZE(), etc.
        function val = INDEX_SIZE(new_value)
            persistent v
            if isempty(v)
                v = 1; %Default value
            end
            if nargin %Set value?
                v = new_value;
            end
            val = v;
        end
    end % RangeImage<-properties(static)
    
    %% PROPERTIES
    properties(GetAccess=public, SetAccess=private)
        raw;    % Vector of Raw Sensor Data.
        
        % Valid Data:
        data = struct( ...
            'ranges', 0, ...    % Vector of Valid Range Data
            'angles', 0, ...    % Vector of Angles Associated with Each Valid Range Pixel
            'xs', 0, ...        % Vector of X-Positions Associated with Each Valid Range Pixel
            'ys', 0 ...         % Vector of Y-Positions Associated with Each Valid Range Pixel
        );
        
    end % RangeImage<-properties(public,private)
    
    %% METHODS
    methods(Access=public)
        %% Constructor:
        % Store relevant data from raw lidar sensor data
        function obj = RangeImage(raw_data)
            % Store Input Data:
            obj.raw = raw_data;
            
            % Process and Store Sensible Information from Raw Data:
            [obj.data.ranges, obj.data.angles] = RangeImage.cleanImage(obj.raw);
            [obj.data.xs, obj.data.ys] = RangeImage.arToXy(obj.data.ranges, obj.data.angles);
            
        end % Constructor
        
        %% Plot:
        % Plots the valid sensor data stored in this instance of
        % RangeImage. Effective drop in for a standard #plot function
        % (doesn't call figure, subplot, &c.)
        % plot_obj - the data will be plotted with the given object instead
        % of creating a new one if this argument is provided.
        % colorize - [bool] if true, a z-axis will be plotted for each
        % point representing it's distance from the robot which can be
        % used for plotting range with color.
        function pl = plot(obj, colorize, plot_obj)
            xs = obj.data.xs;
            ys = obj.data.ys;
            rs = obj.data.ranges;
            
            if nargin>1 % Colorize setting provided
                if(nargin<3) % Create new plot
                    if colorize
                        pl = scatter(ys, xs, 36, rs);
                    else
                        pl = scatter(ys, xs, 36);
                    end
                    set(gca, 'Xdir', 'reverse'); % Ensure Robot Y-Axis Points Left
                else % Use plot handle given
                    pl = plot_obj;
                    if(colorize)
                        set(pl, 'XData', ys, 'YData', xs, 'CData', rs);
                    else
                        set(pl, 'XData', ys, 'YData', xs);
                    end % colorize?
                end % nargin<2?
            else % colorize data not provided and new plot must be made.
                pl = scatter(ys,xs,36);
                set(gca, 'Xdir', 'reverse'); % Ensure Robot Y-Axis Points Left
            end % nargin>1?
        end % #plot
        
        % Locates Candidates for Valid Contiguous Line Segments within the
        % Validated Data Set
        function findLineCandidate(obj)
            
        end
    end % RangeImage<-methods(public)
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% METHODS - STATICS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods (Static)
        %% Clean Image
        % Cleans the Image data by tossing out values above max_rng and 
        % below min_rng. Return vector of valid lidar ranges and their
        % associated angles (in the coord-system referenced at the top).
        function [r_clean,ang] = cleanImage(r_in, min_rng,max_rng)
            r_min = RangeImage.MIN_RANGE;% Default values
            r_max = RangeImage.MAX_RANGE;
            if nargin>2
                r_min = min_rng;
                r_max = max_rng;
            elseif nargin>1
                r_min = min_rng;
            end % nargin?
            
            % Process Range Data:
            valid = r_in>r_min & r_in<r_max;
            r_clean = r_in(valid);
            
            pixels = length(r_in);
            idx = (1:pixels); %Instruction: linspace(2,pixels,pixels)';
            idx = idx(valid);
            
            ang = (idx - 1)*(pi/180) - deg2rad(RangeImage.INDEX_OFFSET());
        end % #cleanImage
        
        %% Index to Bearing
        % Converts a given Range Point Index to a Bearing in the Robot's
        % Reference Frame.
        % returns: th - [deg] bearing in degrees.
        function th = index2bearing(i)
            th = RangeImage.INDEX_OFFSET() + (i-1)*RangeImage.INDEX_SIZE();
            th = mod(th,360);
            th = (th > 180)*(th-360) + (abs(th) <= 180)*th ...
               + (th < -180)*(th+360); %Ensure output ranges from -180 to 180deg.
        end % #index2bearing
        
        %% AR to XY
        % Vectorized function, takes vector rs of Valid Ranges and a vector
        % angs of the Angles Associated with each Valid Range Pixel.
        % Returns:
        % xs - m, X-Position of point (i,r) in robot's reference frame.
        % ys - m, Y-Position of point (i,r) in robot's reference frame.
        function [xs, ys] = arToXy(rs,angs)
            xs = rs .* cos(angs);
            ys = rs .* sin(angs);
        end % #irToXy
         %% IR to XY
        % Finds position and bearing of a range pixel endpoint in the plane.
        % Returns:
        % x,y - [m] position of point (i,r) in robot's reference frame.
        % th - [rad] bearing of point (i,r) in robot's reference frame.
        function [x, y, th] = irToXy_deg(i,r)
            th = RangeImage.index2bearing(i);
            th = deg2rad(th);
            
            x = r*cos(th);
            y = r*sin(th);
        end% #irToXy
        
        %% Plot Range Image
        % Processes Range Data to Plot what the Robot Sees.
        % Returns the handle of the figure the plot is held in.
        %
        % ranges - a list of the distance readings collected from the
        % laser range finder starting from RangeImage.INDEX_OFFSET degrees,
        % which evenly spaced by RangeImage.INDEX_SIZE degrees.
        
        % colorize - [bool] if true, a z-axis will be plotted for each
        % point representing it's distance from the robot which can be
        % used for plotting range with color.
        function f = plot_rangeData(rs, colorize)
        % TODO: Plot (x,y,r) where r is distance for ZData to serve as
        % color.
            persistent fig pl

            % Data Processing
            n = length(rs);
            xs = zeros(1,n);
            ys = zeros(1,n);
            ths = zeros(1,n); % in radians
            for i = (1:length(rs))
                [xs(i), ys(i), ths(i)] = RangeImage.irToXy(i, rs(i));
            end

            % Environment Plotting
            if isempty(fig) %Instantiate plot on first call
                fig = figure();
                if(colorize)
                    pl = scatter(-ys, xs, 36, rs);
                else
                    pl = scatter(-ys, xs);
                end % colorize?
                    axis(2*[-1 1 -1 1])
                    title('LIDAR Data')
                    xlabel('Negative Y-Position [m]')
                    ylabel('X-Position [m]')
                    set(pl, 'XData', -ys)
                    set(pl, 'YData', xs)
                    if colorize; set(pl, 'ZData', rs); end
                refreshdata
                drawnow
            end
            if(colorize)
                set(pl, 'XData', -ys, 'YData', xs, 'ZData', rs) %Update
            else
                set(pl, 'XData', -ys, 'YData', xs) %Update
            end % colorize?
            refreshdata
            drawnow limitrate
            
            f = fig;
        end % #plot_rangeData
    end % RangeImage<-methods(Static)
end %RangeImage Class