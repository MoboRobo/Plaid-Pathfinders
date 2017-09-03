robot = raspbot("Raspbot-4")
robot.stop()

timeArray = zeros(1,1)
leftArray = zeros(1,1)
rightArray = zeros(1,1)
leftStart = robot.encoders.LatestMessage.Vector.X;
pause(.05)
rightStart = robot.encoders.LatestMessage.Vector.Y;
leftEncoder = leftStart; rightEncoder = rightStart;
avgDist = 0;
iterations = 1
totalTimeElapsed = 0
tic
while (avgDist < 30.48) %30.48 cm = 1 in
    pause(.05)
    robot.sendVelocity(.05, .05)
    pause(.05)
    iterations = iterations+1;
    elapsedTime = toc
    pause(.05)
    leftDist = (robot.encoders.LatestMessage.Vector.X - leftStart) * 100.0
    pause(.05)
    rightDist = (robot.encoders.LatestMessage.Vector.Y - rightStart) * 100.0
    avgDist =  (leftDist + rightDist)/2.0
    timeArray(iterations) = toc
    leftArray(iterations) = leftDist
    rightArray(iterations) = rightDist
    plot(timeArray, leftArray, timeArray, rightArray)
end
offset = toc
robot.stop()
pauseTime = 1
while (toc - offset < pauseTime)
    robot.sendVelocity(0, 0)
    pause(.05)
end
while (avgDist >= 0)
    robot.sendVelocity(-.05, -.05)
    pause(.05)
    iterations = iterations+1
    leftDist = (robot.encoders.LatestMessage.Vector.X - leftStart) * 100.0
    pause(.05)
    rightDist = (robot.encoders.LatestMessage.Vector.Y - rightStart) * 100.0
    pause(.05)
    avgDist =  (leftDist + rightDist)/2.0
    timeArray(iterations) = toc
    leftArray(iterations) = leftDist
    rightArray(iterations) = rightDist
    plot(timeArray, leftArray, timeArray, rightArray)
end

robot.stop()