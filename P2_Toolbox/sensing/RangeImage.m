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
    
        
        % Line (Pallet) Candidates within Valid Range Data:
        line_candidates = struct( ...
            'poses', pose(0,0,0), ...   % Poses of Lines in Robot Frame
            'lengths', 0 ...            % Lengths of Lines
        );
        
    end % RangeImage<-properties(public,private)
    
    %% METHODS
    methods(Access=public)
        %% Constructor:
        % Store relevant data from raw lidar sensor data
        function obj = RangeImage(raw_data)
            % Store Input Data:
            % Ensure Raw Data is Row Vector (fixes Robot_API bug):a
            if ~isrow(raw_data)
                obj.raw = raw_data';
            else
                obj.raw = raw_data;
            end
            
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
            
            if ~sum(isnan(rs))
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
            else
                warning('No Data in Range to Plot');
            end
        end % #plot
        
        % Locates Candidates for Valid Contiguous Line Segments of length 
        % approx "length" within the Validated Data Set. Returns the pose 
        % vectors and corresponding lengths of each detected line segment 
        % (pallet).
        function findLineCandidates(obj, l,mle)
        %[obj.line_candidates.poses, obj.line_candidates.lengths] = getPalletPoses(obj, obj.raw);
        
            initialNumSkippedPoints = 0;
            %minimum number of points allowable in point cloud
            minNumPoints = 3;
            
            if nargin>1
                halfSailLength = l/2;
            else
                halfSailLength = 0.0635; %in meters
            end
            if nargin>2
                marginOfLengthError = mle;
            else
                marginOfLengthError = 0.03; %5 centimeters of leeway
            end
            
            len = length(obj.data.ranges); % Number of Valid Range Readings
            
            % Returns cloud of pixels within a certain distance of a given
            % index.
            function [cloudXs, cloudYs, startI, endI] = getPixelsWithin(i, maxDistance)
                midX = getIth(obj.data.xs, i, len);
                midY = getIth(obj.data.ys, i, len);
                midTh = getIth(obj.data.angles, i, len);
                cloudXs = [midX];
                cloudYs = [midY];
                %leftSide
                offset = 1;
                while (1)
                    curX = getIth(obj.data.xs, i - offset, len);
                    curY = getIth(obj.data.ys, i - offset, len);
                    curTh = getIth(obj.data.angles, i - offset, len);
                    if ( norm([curX-midX, curY-midY]) > maxDistance )
                        break;
                    end
                    cloudXs = [curX cloudXs];
                    cloudYs = [curY cloudYs];
                    offset = offset+1;
                end
                offset = 1;
                %rightSide
                while(1)
                    curX = getIth(obj.data.xs, i + offset, len);
                    curY = getIth(obj.data.ys, i + offset, len);
                    curTh = getIth(obj.data.angles, i + offset, len);
                    if ( norm([curX-midX, curY-midY]) > maxDistance )
                        break
                    end
                    cloudXs(end+1) = curX;
                    cloudYs(end+1) = curY;
                    offset = offset+1;
                end
            end % #getPixelsWithin
            
            %Gets a Segment of Continuous Pixels
            function [cloudXs, cloudYs, startI, endI] = getContinuousPixels(i)
                startI = i; endI = i;
                rawLen = length(obj.raw);
                midX = getIth(obj.data.xs, i, len);
                midY = getIth(obj.data.ys, i, len);
                midTh = getIth(obj.data.angles, i, len);
                cloudXs = [midX];
                cloudYs = [midY];
                %leftSide
                offset = 1;
                prevX = midX; prevY = midY;
                numSkippedPoints = initialNumSkippedPoints;
                curRadius = getIth(obj.data.ranges, i-offset, len);
                while (1)
                    %get current radius to figure maxDist
                    maxDistance = (curRadius*2*pi / double(rawLen)) * 1.5;
                    curX = getIth(obj.data.xs, i - offset, len);
                    curY = getIth(obj.data.ys, i - offset, len);
                    curTh = getIth(obj.data.angles, i - offset, len);
                    if ( norm([curX-prevX, curY-prevY]) > maxDistance)
                        if (numSkippedPoints == 0)
                            break;
                        else
                            numSkippedPoints = numSkippedPoints -1;
                            curRadius = curRadius*2;
                            continue;
                        end
                    end
                    prevX = curX; prevY = curY;
                    cloudXs = [curX cloudXs];
                    cloudYs = [curY cloudYs];
                    startI = i - offset;
                    offset = offset+1;
                    curRadius = getIth(obj.data.ranges, i-(offset), len);
                end
                offset = 1;
                prevX = midX; prevY = midY;
                %rightSide
                numSkippedPoints = initialNumSkippedPoints;
                curRadius = getIth(obj.data.ranges, i+offset, len);
                while(1)
                    maxDistance = (curRadius*2*pi / double(rawLen)) * 1.5;
                    curX = getIth(obj.data.xs, i + offset, len);
                    curY = getIth(obj.data.ys, i + offset, len);
                    curTh = getIth(obj.data.angles, i + offset, len);
                    if ( norm([curX-prevX, curY-prevY]) > maxDistance)
                        if (numSkippedPoints == 0)
                            break;
                        else
                            numSkippedPoints = numSkippedPoints -1;
                            curRadius = curRadius*2;
                            continue;
                        end
                    end
                    
                    prevX = curX; prevY = curY;
                    cloudXs(end+1) = curX;
                    cloudYs(end+1) = curY;
                    offset = offset+1;
                    curRadius = getIth(obj.data.ranges, i+(offset), len);
                    endI = i + offset;
                end
            end % #getPixelsWithin
            
            % OBSOLETE:
            % Rejects points from given cloud which are not near the bulk
            % of the points of interest (i.e. bits of a wall or other pallet 
            % behind the pallet of interest).
            function [bulkXs, bulkYs] = rejectOutliers(cXs, cYs)
            % This algorithm just rejects points whose distances are
            % more than 1.5*IQR beyond Q1,Q3. This is not super robust as
            % a large wall segment behind a pallet could shift the mean over
            % to the wall.
                % Distances to each point in the cloud:
                cPs = [cXs; cYs]; % Cloud Points
                cSs = sqrt(sum(cPs.^2,1));
                
                Q1S = prctile(cSs,25);
                Q3S = prctile(cSs,75);
                rngS = 1.5*iqr(cSs);
                
                bulkXs = []; bulkYs = [];
                
                n = length(cSs);
                i = 1;
                while i<=n
                    if( cSs(i) > (Q1S-rngS) && cSs(i) < (Q3S+rngS) )
                        bulkXs(end+1) = cXs(i);
                        bulkYs(end+1) = cYs(i);
                    end
                i = i+1;
                end
                
            end % rejectOutliers
            
            pixelIndex = 1;
            while pixelIndex <= len
                
                % *Have search window be larger than w_sail so that larger
                % widths can be observed and rejected.
                % (probably should also be smaller than 2*w_sail so the
                % mean isn't shifted over to any outliers causing the
                % pallet to be rejected. (Also less pts -> faster).
%                 search_radius = 1.45*halfSailLength + marginOfLengthError/2;
%                 [cloudXs, cloudYs] = getPixelsWithin(pixelIndex, search_radius);
                %[cloudXs, cloudYs] = rejectOutliers(cloudXs, cloudYs);
                [cloudXs, cloudYs, sI, eI] = getContinuousPixels(pixelIndex);
                
                numPoints = length(cloudXs);
                if numPoints < minNumPoints
                    %skip to next iteration
                    break;
                end
                
%                 originX = getIth(obj.data.xs, pixelIndex, len);
%                 originY = getIth(obj.data.ys, pixelIndex, len);
                
                centerX = sum(cloudXs) / numPoints;
                centerY = sum(cloudYs) / numPoints;
                
                % Amount Center of Data has Shifted from where Search was
                % Originated.
%                 origin_shift = norm([originX-centerX, originY-centerY]);
                
                %Center points around origin
                cloudXs = cloudXs - centerX;
                cloudYs = cloudYs - centerY;

                Ixx = cloudXs * cloudXs';
                Iyy = cloudYs * cloudYs';
                Ixy = - cloudXs * cloudYs';
                inertia = [Ixx Ixy; Ixy Iyy] / length(cloudXs);
                lambda = eig(inertia);
                lambda = sqrt(lambda) * 1000.0; % in mm
                
                estimatedLength = norm([cloudXs(1)-cloudXs(end), ...
                                        cloudYs(1)-cloudYs(end)]);
%                 estimatedLength = sqrt(max(eig([Ixx Ixy; Ixy Iyy])));
                                    
                % CONDITIONS FOR VALID LINE CANDIDATE:
                if( lambda(1) < 1.3 ...
                && (abs(estimatedLength - halfSailLength*2) < marginOfLengthError) ...
                )
%                 && origin_shift < halfSailLength/4 )
                
                    new_th = atan2(2*Ixy, Iyy-Ixx) / 2.0;
                    new_x = centerX;
                    new_y = centerY;
                    
                    % When pallet is at X<0, th points towards robot
                    % instead of away, causing acquisition trajectories to
                    % require Figure-8. Fix this by flipping th if X<0.
                    if(new_x < 0)
                        new_th = new_th + pi;
                        new_th = atan2(sin(new_th),cos(new_th));
                    end
                    
                    new_pose = pose(new_x, new_y, new_th);
                    
                    obj.line_candidates.poses = [obj.line_candidates.poses new_pose];
                    obj.line_candidates.lengths = [obj.line_candidates.lengths estimatedLength];
                end
            %pixelIndex = pixelIndex+1;
            
            pixelIndex = max(pixelIndex+1, eI+1);
            end
            
            % Returns the Ith Data from the Array while Ensuring
            % Wrap-Around
            function elem = getIth(array, i, len)
                elem = array( mod(i-1, len) + 1);
            end % #getIth
        end
        
        % Plots Every Line Candidate within the Validated Data Set (in the 
        % Robot Frame).
        % Optional Argument: - cp: clears prev plotting of line
        % candidates
        function plotLineCandidates(obj, cp)
        persistent plot_objs % Objects Plotted.
            
            clear_prev = 1; % Default value
            if nargin>1
                clear_prev = cp;
            end
            
            % Clear Previous Plots (i/a):
            if( ~isempty(plot_objs) && clear_prev )
                i = 1;
                while i<=length(plot_objs)
                    p = plot_objs(i);
                    try
                        delete(p);
                    catch err
                        % object deleted elsewhere (likely by plot
                        % termination
                    end
                i = i+1;
                end
            end % isempty(plot_objs)&clear_prev?
            
            % Pre Allocate
            n = length(obj.line_candidates.lengths);
            plot_objs(1:n) = 0;
            
            i = 1;
            hold on
            while i<=n
                p = obj.line_candidates.poses(i);
                l = obj.line_candidates.lengths(i);

                th = p.th;

                x0 = p.X - l*cos(th)/2;
                x1 = p.X + l*cos(th)/2;

                y0 = p.Y - l*sin(th)/2;
                y1 = p.Y + l*sin(th)/2;
                plot_objs(i) = plot([y0 y1], [x0 x1]);
            i = i+1;
            end
            hold off
            set(gca, 'Xdir', 'reverse'); % Ensure Robot Y-Axis Points Left
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