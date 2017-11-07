classdef lineMapLocalizer < handle
     %mapLocalizer A class to match a range scan against a map in
     % order to find the true location of the range scan relative to
     % the map.

     properties(Constant)
        maxErr = 0.05; % 5 cm
        minPts = 5; % min # of points that must match
     end

     properties(Access = private)
     end

     properties(Access = public)
        lines_p1 = [];
        lines_p2 = [];
        gain = 0.3;
        errThresh = 0.01;
        gradThresh = 0.0005;
     end

     methods
        function obj = ...
            lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
            % create a lineMapLocalizer
            obj.lines_p1 = lines_p1;
            obj.lines_p2 = lines_p2;
            obj.gain = gain;
            obj.errThresh = errThresh;
            obj.gradThresh = gradThresh;
        end

        function ro2 = closestSquaredDistanceToLines(obj,pi)
            % Find the squared shortest distance from pi to any line
            % segment in the supplied list of line segments.
            % pi is an array of 2d points
            % throw away homogenous flag
            pi = pi(1:2,:);
            r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,...
                obj.lines_p1(:,i),obj.lines_p2(:,i));
            end
            ro2 = min(r2Array,[],1);
        end

        function ids = throwOutliers(obj,pose,ptsInModelFrame)
            % Find ids of outliers in a scan.
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            ids = find(r2 < obj.maxErr*obj.maxErr);
        end

        function avgErr2 = fitError(obj,pose,ptsInModelFrame)
            % Find the variance of perpendicular distances of
            % all points to all lines

             % transform the points
            worldPts = pose.bToA()*ptsInModelFrame;
            %try

            r2 = obj.closestSquaredDistanceToLines(worldPts);
            r2(r2 == Inf) = [];
            err2 = sum(r2);
            num = length(r2);
            if(num >= lineMapLocalizer.minPts)
                avgErr2 = err2/num;
            else
            % not enough points to make a guess
            avgErr2 = inf;
            end
        end

        function [currErr,J] = getJacobian(obj,poseIn,modelPts)
            % Computes the gradient of the error function
            currErr = fitError(obj,poseIn,modelPts);
            eps = 1e-6;
            dx = [eps ; 0.0 ; 0.0];
            dxPoseErr = fitError(obj,pose(poseIn.poseVec()+dx),modelPts);
            dE_dx = (1/eps)*(dxPoseErr-currErr);

            dy = [0.0; eps; 0.0];
            dyPoseErr = fitError(obj,pose(poseIn.poseVec()+dy),modelPts);
            dE_dy = (1/eps)*(dyPoseErr-currErr);

            dTheta = [0.0; 0.0; eps];
            dthPoseErr = fitError(obj,pose(poseIn.poseVec()+dTheta),modelPts);
            dE_th = (1/eps)*(dthPoseErr-currErr);
            currTheta = poseIn.poseVec(3);
            newTheta = atan2(sin(currTheta + dTheta), cos(currTheta + dTheta));
            
            newPose = pose(poseIn.poseVec(1),poseIn.poseVec(2),newTheta);

            dThetaPoseErr = fitError(obj,newPose,modelPts);
            dE_dTheta = (1/eps)*(dThetaPoseErr-currErr);

            J = [dE_dx; dE_dy; dE_dTheta]; 
        end

        function [success, ret_curpose, ptsAnalysed]...
            = refinePose(obj, inPose, ptsInModelFrame, maxIterations)

            success = 0;
            % refine robot pose in world (inPose) based on lidar
            % registration. Terminates if maxIters iterations is
            % exceeded or if insufficient points match the lines.
            % Even if the minimum is not found, outPose will contain 
            % any changes that reduced the fit error. Pose changes that
            % increase fit error are not included and termination
            % occurs thereafter.
            xs = []; ys = [];
            %thresholds taken from values recommended in lab writeup
            % should I throw values as they do in write up with worldPts(:,ids) =
            % yes I should, so I am below:)
            %ptsInModelFrame = obj.throwOutliers(inPose, ptsInModelFrame);
            curpose = inPose.poseVec();
            % compute gradient, determine how far away from desired position we are
            
            ids = obj.throwOutliers(inPose, ptsInModelFrame);
            ptsInModelFrame = ptsInModelFrame(:,ids);
            ptsAnalysed = ptsInModelFrame;
            [curErr, J] = obj.getJacobian(pose(curpose), ptsInModelFrame);
         %   fprintf('initial fit error: %d\n', curErr);
            lastErr = 1000; lastMagOfJ = 10000;

            outpose = curpose;
            for i = 1:maxIterations
                %move small amount along negative gradient
                curpose = curpose - obj.gain * J;
                xs = [xs curpose(1)]; ys = [ys curpose(2)];


                squaredVals = J .* J; sumSquaredVals = sum(squaredVals); 
                magnitudeOfJ = sqrt(sumSquaredVals);
                %after changes, recompute error accordingly
                [curErr, J] = obj.getJacobian(pose(curpose), ptsInModelFrame);
%                 title(sprintf('Fit Error: %d Iteration no. %d', curErr, i)); 
                %if error is small or gradient is near zero magnitude you're done
                if (curErr < obj.errThresh )%|| magnitudeOfJ < obj.gradThresh)
%                     fprintf('happened\n');
                    outpose = curpose;
                    success = 1;
                    break
                elseif (curErr > lastErr)
                    break
                end
%                  fprintf('curError: %d\n', curErr);
%                 pause(.001);
                outpose = curpose; lastErr = curErr; lastMagOfJ = magnitudeOfJ;
            end
%             figure 
%             hold on
%             plot(obj.lines_p1, obj.lines_p2)
%             scatter(xs, ys);
%             xlabel('X'); ylabel('Y');
        ret_curpose = pose(outpose);
        end
     end
end