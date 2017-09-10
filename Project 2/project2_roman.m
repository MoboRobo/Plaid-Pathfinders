robot = raspbot("Raspbot-4")
robot.stopLaser()
pause(1)
robot.startLaser()

desiredDistance = .5
buffer = .05
while (true)
    index = 0
    Xs = []
    Ys = []
    pause(.2)
    readings = robot.laser.LatestMessage.Ranges
    indicesOfInterest = readings>.06 & readings<1.0
    indexArray = 1:360
    filteredReadings = readings(indicesOfInterest)
    indexArray = indexArray(indicesOfInterest)
    indicesOfInterest = indexArray > 270 | indexArray < 90
    indexArray = indexArray(indicesOfInterest)
    filteredReadings = filteredReadings(indicesOfInterest)
    [Xs Ys something] = irToXyOnArray(indexArray, filteredReadings')
%     scatter(Xs, Ys)
%     axis([-2 2 -2 2])
%     xlabel("Distance in X axis (meters)")
%     ylabel("Distance in Y axis (meters)")
    [closest, index] = min(filteredReadings)
    index = indexArray(index)
    if not (closest > desiredDistance - buffer & closest < desiredDistance-buffer)
        gain = (closest-desiredDistance)
        scaleFactor = .8
        if gain < -.5
            gain = -.5
        elseif gain > .5
            gain = .5
        end
        %calculate curvature and set velocity
        K = (sin(deg2rad(index)) / (closest * sin(deg2rad(90-index))))*5 
        V = gain*scaleFactor
        angularVelocity = K * V
        wheelDistance = .08 %in meters
        v_left = V - (wheelDistance/2)*angularVelocity
        v_right = V + (wheelDistance/2)*angularVelocity
        robot.sendVelocity(v_right, v_left)
    [x y th] = irToXy(index, closest)
    
    scatter([x], [y])
    axis([-2 2 -2 2])
    xlabel("Distance in X (meters)")
    ylabel("Distance in Y (meters)")
    end
    
%     for i = 1:360
%         r = arr(i)
%         index = index + 1
%         [x y th] = irToXy(i, r)
%         Xs(index) = x
%         Ys(index) = y
%         plot(Xs, Ys)
%     end
end
            
function [x y th] = irToXyOnArray (i, r)
th = (i-1+90) * (1/360) * 2* pi
x = r .* cos(th)
y = r .* sin(th)
end

function [x y th] = irToXy (i, r)
th = (i-1+90) * (1/360) * 2* pi
x = r * cos(th)
y = r * sin(th)
end