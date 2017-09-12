% Container Class for Manipulating Laser Range Finder (~Lidar) Data.
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
classdef RangeImage
    %% STATIC PROPERTIES
    % Must be handled as persistent variables wrapped in an accessor which
    % returns the value and sets the value if given an argument.
    methods (Static)
        % [deg] Offset of index 0 in the range data to 0 bearing on the 
        % robot (+ve offset -> index 0 starts on robot's left side).
        function val = INDEX_OFFSET(new_value)
            persistent v
            if isempty(v)
                v = 0; %Default value
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
    end % RangeImage->methods(static)
    
    %% METHODS
    methods (Static)
        %% Clean Image
        % Cleans the Image data by tossing out values above max_rng and 
        % below min_rng. All invalid data is zeroed.
        function r_clean = cleanImage(r_in, min_rng,max_rng)
            r_clean = zeros(1,length(r_in));
            for i = (1:length(r_in))
                r_clean(i) = (r_in(i)<max_rng && r_in(i)>min_rng)*r_in(i);
            end 
        end % #cleanImage
        
        %% Index to Bearing
        % Converts a given Range Point Index to a Bearing in the Robot's
        % Reference Frame.
        % returns: th - [deg] bearing in degrees.
        function th = index2bearing(i)
            th = RangeImage.INDEX_OFFSET() + (i-1)*RangeImage.INDEX_SIZE();
        end % #index2bearing
        
        %% IR to XY
        % Finds position and bearing of a range pixel endpoint in the plane.
        % Returns:
        % x,y - [m] position of point (i,r) in robot's reference frame.
        % th - [rad] bearing of point (i,r) in robot's reference frame.
        function [x, y, th] = irToXy(i,r)
            th = RangeImage.index2bearing(i);
            th = mod(th,360);
            th = (th > 180)*(th-360) + (abs(th) <= 180)*th ...
               + (th < -180)*(th+360); %Ensure output ranges from -180 to 180deg.
            th = deg2rad(th);
            
            x = r*cos(th);
            y = r*sin(th);
        end % #irToXy
        
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
    end % RangeImage->methods(Static)
end %RangeImage Class