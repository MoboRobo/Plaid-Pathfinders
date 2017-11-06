function [success, curpose]...
    = refinePose(obj, inPose, ptsInModelFrame, maxIterations)

    success = 0;
    
    % refine robot pose in world (inPose) based on lidar
    % registration. Terminates if maxIters iterations is
    % exceeded or if insufficient points match the lines.
    % Even if the minimum is not found, outPose will contain 
    % any changes that reduced the fit error. Pose changes that
    % increase fit error are not included and termination
    % occurs thereafter.
    
    %thresholds taken from values recommended in lab writeup
    gradientThreshold = .0005; errThreshold = .01;
    % should I throw values as they do in write up with worldPts(:,ids) =
    % []?
    k = .1;
    curpose = inPose;
    % compute gradient, determine how far away from desired position we are
    [curErr, J] = obj.getJacobian(curpose, ptsInModelFrame);
    lastErr = curErr; lastMagOfJ = norm(J);
    for i = 1:maxIterations
        %move small amount along negative gradient
        curpose = curpose - k * J;
        
        
        
        squaredVals = J .* J; sumSquaredVals = sum(squaredVals); 
        magnitudeOfJ = sqrt(sumSquaredVals);
        %after changes, recompute error accordingly
        [curErr, J] = obj.getJacobian(curpose, ptsInModelFrame);
        %if error is small or gradient is near zero magnitude you're done
        if (curErr < errThreshold || magnitudeOfJ < gradientThreshold)
            outpose = curpose;
            success = 1;
            break
        elseif (curErr > lastErr || lastMagOfJ > magnitudeOfJ)
            break
        end
       
        outpose = curpose; lastErr = curErr; lastMagOfJ = magnitudeOfJ;
    end

end