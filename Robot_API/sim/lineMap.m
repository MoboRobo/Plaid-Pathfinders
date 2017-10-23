classdef lineMap < handle
    %lineMap environment map made up of lineObjects
    
    properties (Constant = true)
        sampleResln = 0.05; % cm
        doIncidence = false; % added by Al Nov 20, 2016
    end
    
    properties (GetAccess = public, SetAccess = private)
		num_objects = 0;
		objects;
    end
    
    methods (Access = public)
        function lm = lineMap(objects)
            if( nargin < 1)
                error('You need to intialize lineMap with at least one object');
            end
            
            for obj = objects
                lm.addObject(obj);
            end
		end
        
		function n = addObject(lm, object)
			if( isempty(object.lines))
				error('Object must have line of size n x 2 (n>0)');
			else				
				if ~isempty(object.path)
					object.has_path = true;
					object.pose = object.path(1,1:3)';
				else
					object.has_path = false;
				end
				object.startTimer();
				
				object.update();

				if isempty(lm.objects)
					lm.objects = object;
				else
					lm.objects(end + 1) = object;
				end
				lm.num_objects = lm.num_objects + 1;
				lm.objects(end).id = lm.num_objects;
				n = lm.num_objects;
			end
        end
        
        function hfig = plot(obj)
           hfig = figure; hold on;
           for object = obj.objects 
               object.update();
               object.plot();
           end
           axis equal
           hold off
        end
		
        function [ranges,incidence_angles] = raycast(obj, pose, max_range, ang_range)
            %RAYCAST
            %
            % [ranges,incidence_angles] = RAYCAST(obj, pose, max_range, ang_range)
            %
            %  pose            - Length 3 array.
            %  max_range       - In m.
            %  ang_range       - Vector in radian.
            %
            % ranges           - Vector in m.
            % incidence_angles - Vector in rad.
            
            sweep = pose(3) + ang_range;
            
            p0x = pose(1);
            p0y = pose(2);
            
            p1x = p0x + max_range.*cos(sweep);
            p1y = p0y + max_range.*sin(sweep);
			
            s1x = p1x - p0x;
            s1y = p1y - p0y;
            
            lc = cell2mat({obj.objects(:).line_coords}');
            
            % start points of lines
            p2x = lc(1:end-1,1);
            p2y = lc(1:end-1,2);
            
            % end points of lines
            p3x = lc(2:end,1);
            p3y = lc(2:end,2);
            
            % line segments
            s2x = p3x - p2x; 
            s2y = p3y - p2y;

            % s takes into account that the line segment is finite
            s = ((p2x-p0x)*s1y + (p0y-p2y)*s1x)./...
                (-s2x*s1y + s2y*s1x);
            
            % t takes into account that the rays are finite
			t = repmat(( s2x.*(p0y-p2y) - s2y.*(p0x-p2x)),1,length(s1x))...
                ./ (-s2x*s1y + s2y*s1x);

            col = s >= 0 & s <= 1 & t >= 0 & t <= 1;
            t(~isfinite(t)) = 0;
            cpx = (t .* repmat(s1x,length(s2x),1)) .* col;
            cpy = (t .* repmat(s1y,length(s2x),1)) .* col;
            r = (cpx.^2 + cpy.^2).^.5;
            
            r(~col) = nan;
            
            % angles of incidence to each line segment
            % currently has positive/ negative ambiguity
            if(lineMap.doIncidence)
                alpha = zeros(length(p2x),1);
                for i = 1:length(p2x)
                    params = ParametrizePts2ABC(lc(i,1:2),lc(i+1,1:2));
                    alpha(i) = atanLine2D(params(1),params(2));
                end
                incidence_angles = repmat(alpha,1,length(ang_range))-repmat(mod(ang_range+pose(3),pi),length(alpha),1); 
                incidence_angles(incidence_angles == 0) = pi;
                incidence_angles = incidence_angles-pi/2*ones(size(incidence_angles)).*sign(incidence_angles);
            end                                
            % remove spurious line segments between points not belonging to same
            % object
            ng = cumsum(cellfun(@(x) size(x,1), {obj.objects(:).line_coords}'));
            r(ng(1:end-1),:) = [];
            if(lineMap.doIncidence)
                incidence_angles(ng(1:end-1),:) = [];
            end
            % get minimum range and angle to that segment
            [ranges, min_sub] = min(r);
            if(lineMap.doIncidence)
                ids = sub2ind(size(incidence_angles),min_sub,1:length(ang_range));
                incidence_angles = incidence_angles(ids);
                incidence_angles(isnan(ranges)) = nan;
                ranges( isnan(ranges) ) = 0;

                % return absolute value of incidence angles
                incidence_angles = abs(incidence_angles);
            else
                incidence_angles = zeros(size(ranges));
            end
        end
        
        function [ranges,angles] = raycastNoisy(lm, pose, max_range, ang_range,choice)
            % wrapper around raycast to introduce noise
            if nargin < 5
                choice = 1;
            end
            if choice == 1
                % add gaussian noise with stddev based on range and angle of
                % incidence
                [ranges,angles] = raycast(lm, pose, max_range, ang_range);
                
                K = 1e-3;
                for i = 1:length(ranges);
                    sigma = K*ranges(i)^2/cos(angles(i));
                    ranges(i) = ranges(i)+sigma*randn;
                end
            end
        end
                
		function [dist, ob_num, line_num] = closestObject(lm, pos)

            lc = cell2mat({lm.objects(:).line_coords}');
			
			if( all(size(pos) == [1 2]) )
				P = pos;
			elseif( all(size(pos) == [2 1]) )
				P = pos';
			else
				error('pos must be 2x1');
			end
			
			%line endpoints
			P0 = lc(1:end-1,:);
			P1 = lc(2:end,:);
			
			%Query Point
			P = repmat(P, size(P0,1),1);
			
			D = zeros(size(P0,1),1);
			
			%useful vectors
			v = P1 - P0;
			w = P - P0;

			%dot products to check case
			c1 = sum(v.*w,2);
			c2 = sum(v.*v,2);
			
			%before P0
			bp0 = c1<=0;
			D(bp0) = sqrt(sum((P(bp0,:) - P0(bp0,:)).^2,2));
			
			%after P1
			ap1 = c2<=c1;
			D(ap1) = sqrt(sum((P(ap1,:) - P1(ap1,:)).^2,2));
			
			%perpedicular to line
			per = ~(bp0 | ap1);
			
			b = c1 ./ c2;
			Pb = P0 + ([b b].*v);
			D(per) = sqrt(sum((P(per,:) - Pb(per,:)).^2,2));
					
			%superfluous (nonexistent lines)
			ng = cumsum(cellfun(@(x) size(x,1), {lm.objects(:).line_coords}'));
			D(ng(1:end-1)) = inf;

			%get minimum distance
			[dist, ind] = min(D);
			
			%find the object and line number
			zng = [0;ng];
			ob_num = find(ind > zng, 1, 'last');
			line_num = ind - zng(ob_num);
        end
		
        function lm = removeObject(lm, i)
			lm.objects(i).lines = [];
			delete(lm.objects(i).h)
			lm.num_objects = lm.num_objects - 1;
		end
				
		function lm = update(lm)
			for i = 1:length(lm.objects)
				lm.objects(i).update();
				lm.objects(i).plot();
			end
        end
        
        function pcd = getPCD(lm,resln)
            if nargin < 2
                resln = lineMap.sampleResln;
            end
            pts = lineMap.sampleFromLines(lm.objects,resln);
            pcd = [pts zeros(size(pts,1),1)]';
        end
    end
    
    methods (Static = true)
        function pts = sampleFromLines(lineArray,resln)
            % lines is an array of lineObject instances
            % pts is n x 2
            
            pts = [];
            nLines = length(lineArray);
            for i = 1:nLines
                lines = lineArray(i).lines;
                for j = 1:size(lines,1)-1;
                    p1 = lines(j,:);
                    p2 = lines(j+1,:);
                    p12 = p2-p1;
                    temp = [0:resln/norm(p12):1]'*p12;
                    temp = bsxfun(@plus,temp,p1);
                    pts = [pts; temp];
                end
            end

        end
    end
end













