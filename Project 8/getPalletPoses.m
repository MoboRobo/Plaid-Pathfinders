function [palletPoses palletLengths] = getPalletPoses(obj, laserEncoderData)
    %minimum number of points allowable in point cloud
    minNumPoints = 3;
    marginOfLengthError = .03; %3 centimeters of leeway
    
    inBounds = laserEncoderData > .06 & laserEncoderData < 4.0;
    inBoundData = laserEncoderData(inBounds);
    indices = 1:360;
    indices = indices(inBounds);
    len = length(inBoundData);
    [xCoords yCoords ths] = irToXyOnArray(indices, inBoundData');
    halfSailLength = .635; %in meters
    function [cloudXs cloudYs] = getPixelsWithin(i, maxDistance)
        midX = getIth(xCoords, i, len);
        midY = getIth(yCoords, i, len);
        midTh = getIth(ths, i, len);
        cloudXs = [midX];
        cloudYs = [midY];
        %leftSide
        offset = 1;
        while (1)
            curX = getIth(xCoords, i - offset, len);
            curY = getIth(yCoords, i - offset, len);
            curTh = getIth(ths, i - offset, len);
            if ( distanceBetween(curX, curY,midX, midY)  > maxDistance )
                break
            end
            cloudXs = [curX cloudXs];
            cloudYs = [curY cloudYs];
            offset = offset+1;
        end
        offset = 1;
        %rightSide
        while(1)
            curX = getIth(xCoords, i + offset, len);
            curY = getIth(yCoords, i + offset, len);
            curTh = getIth(ths, i + offset, len);
            if ( distanceBetween(curX, curY, midX, midY)  > maxDistance )
                break
            end
            cloudXs(end+1) = curX;
            cloudYs(end+1) = curY;
            offset = offset+1;
        end
    end
    for pixelIndex = 0:len
        [cloudXs cloudYs] = getPixelsWithin(pixelIndex, halfSailLength);
        numPoints = length(cloudXs);
        if numPoints < minNumPoints
            %skip to next iteration
            break;
        end
        centerX = sum(cloudXs) / numPoints;
        centerY = sum(cloudYs) / numPoints;
        %get points centered around origin
        cloudXs = cloudXs - centerX;
        cloudYs = cloudYs - centerY;
        
        
        Ixx = cloudXs' * cloudXs;
        Iyy = cloudYs' * cloudYs;
        Ixy = - cloudXs' * cloudYs;
        inertia = [Ixx Ixy; Ixy Iyy] / length(cloudXs);
        lambda = eig(inertia);
        lambda = sqrt(lambda) * 1000.0;
        estimatedLength = distanceBetween(cloudXs(1), cloudYs(1), ...
            cloudXs(end), cloudYs(end));
        if(lambda(1) < 1.3 && (abs(estimatedLength - halfSailLength*2) < ...
                marginOfLengthError))
            palletPoses = [palletPoses [centerX centerY ...
                atan2(2*Ixy, Iyy-Ixx) / 2.0]];
            palletLengths = [palletLengths estimatedLength];
        end
    end



end

function dist = distanceBetween(x0, y0, x1, y1)
    dist = sqrt((x1 - x0)^2 + (y1 - y0)^2)
end


function elem = getIth(array, i, len)
    elem = array(mod(i - 1, len))
end