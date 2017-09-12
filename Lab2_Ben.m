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
minObjectRange = 0.06
idealObjectRange = 0.5

while(iterations < 200)
    rangeArray = robot.laser.LatestMessage.Ranges
    filter = rangeArray > minObjectRange & rangeArray < maxObjectRange
    filterRange = rangeArray(filter)
    filterIndex = rangeArray(filter)
    
    [minRange, minIndex] = min(filterRange)
    
    [xArray, yArray,thArray] = irToXy(iArray,rangeArray') 
    

    
    plot(xArray(filter),yArray(filter),'X')
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

