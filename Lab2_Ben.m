robot = raspbot("Raspbot-4")

robot.startLaser()
pause(2)
iterations = 0

rangeArray = zeros(1,360)
xArray = zeros(1,360)
yArray = zeros(1,360)
thArray = zeros(1,360)

iArray = zeros(1,360)

iArray = 1:360 

maxObjectRange = 1.0
minObjectRange = 0.1 % was 0.06
idealObjectRange = 0.5 %was 0.5

while(iterations < 400)
    rangeArray = robot.laser.LatestMessage.Ranges
    frontFilter = iArray > 270 | iArray < 90
    iArray180 = iArray(frontFilter)
    rangeArray180 = rangeArray(frontFilter)
    
    [xArray, yArray,thArray] = irToXy(iArray180,rangeArray180') 
    
    filter = rangeArray180 > minObjectRange & rangeArray180 < maxObjectRange 
    filterRange = rangeArray180(filter)
    filterIndex = iArray180(filter)
    filterX = xArray(filter)
    filterY = yArray(filter)
    filterTh = thArray(filter)
    
    
    [minRange, minIndex] = min(filterRange)
    
    plot(filterX(minIndex),filterY(minIndex),'X')
    axis([-1 1 -1 1])

    if(isempty(filterRange) == true)
        robot.sendVelocity(0.0, 0.0)
    elseif(minRange < idealObjectRange)
        robot.sendVelocity(-0.1, -0.1)
    else
        pError =  ((pi/2)-filterTh(minIndex))
        %Object Left = positive, Object Right = negative
   
        kp = 0.04
        leftVel = max([min([0.1-kp*pError, 0.4]),0.0])
        rightVel = max([min([0.1+kp*pError, 0.4]),0.0])
        robot.sendVelocity(leftVel, rightVel)
    end
    iterations = iterations + 1
    pause(0.05)
end
robot.stopLaser()
robot.stop()
robot.shutdown()

function [ x y th] = irToXy( i, r )
    % irToXy finds position and bearing of a range pixel endpoint
    % Finds the position and bearing of the endpoint of a range pixel in the plane.
    th = deg2rad(i + 90)
    x = r .* cos(th)
    y = r .* sin(th)
    
end

