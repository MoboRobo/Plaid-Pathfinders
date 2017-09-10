robot = raspbot("Raspbot-17")

robot.startLaser()
pause(2)
iterations = 0
while(iterations < 2)
    %rangeArray = robot.laser.LatestMessage.Ranges
    %rangeArray = zeros(1,360)
    xArray = zeros(1,360)
    yArray = zeros(1,360)
    thArray = zeros(1,360)
    [xArray,yArray,thArray] = irToXy(1,rangeArray)
    scatter(xArray,yArray)
    iterations = iterations + 1
    pause(0.2)
end
robot.stopLaser()

function [ x y th] = irToXy( i, r )
    % irToXy finds position and bearing of a range pixel endpoint
    % Finds the position and bearing of the endpoint of a range pixel in the plane.
    th = deg2rad(i)
    x = r .* cos(th)
    y = r .* sin(th)
    
end

