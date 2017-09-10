robot = raspbot("Raspbot-4")

robot.startLaser()
pause(2)
iterations = 0

rangeArray = zeros(1,360)
xArray = zeros(1,360)
yArray = zeros(1,360)
thArray = zeros(1,360)
maxObjectRange = 1.0
iArray = zeros(1,360)

iArray = 1:360 

minObjectRange = 0.06

while(iterations < 10)
    rangeArray = robot.laser.LatestMessage.Ranges
    
    %[xArray, yArray,thArray] = irToXy(iArray,rangeArray) %doesnt work
     for i = 1:numel(rangeArray) %slow
        [xArray(i), yArray(i),thArray(i)] = irToXy(i,rangeArray(i))
     end
    %{
    filteredX = xArray
    filteredY = yArray
    
    for i = 1:numel(rangeArray)
        if(rangeArray(i) > maxObjectRange | rangeArray(i) < minObjectRange)
            filteredX(i) = NaN
            filteredY(i) = NaN
        end
    end
    %}
    
    minRange = 100
    minIndex  = 0
    for i = 1:numel(rangeArray)
        if(rangeArray(i) < minRange && rangeArray(i) > minObjectRange)
            minRange = rangeArray(i)
            minIndex = i
        end
    end
    
    if(minRange < 0.2 | minRange > maxObjectRange)
        %robot.sendVelocity(0.0, 0.0)
    else
        pError =  xArray(minIndex) 
        %Object Left = positive, Object Right = negative
        kp = 0.2
        leftVel = max([min([0.1+kp*pError, 0.5]),0.0])
        rightVel = max([min([0.1-kp*pError, 0.5]),0.0])
        %robot.sendVelocity(leftVel, rightVel)
    end
 
    plot(xArray(minIndex),yArray(minIndex),'X')
    axis([-1 1 -1 1])
    iterations = iterations + 1
    pause(0.05)
end
robot.stopLaser()
robot.stop()
robot.shutdown()

function [ x y th] = irToXy( i, r )
    % irToXy finds position and bearing of a range pixel endpoint
    % Finds the position and bearing of the endpoint of a range pixel in the plane.
    th = deg2rad(i + 85)
    x = r .* cos(th)
    y = r .* sin(th)
    
end

