tic
while(toc < 5)
    robot.sendVelocity(.050, .050)
    pause(0.05)
end

% Look at the encoder data of the right wheel (in mm)

robot.encoders.LatestMessage.Vector.X