function Lab8_TestBench(robot_id)
    rasp = raspbot(robot_id);
    rasp.laser.NewMessageFcn = @processLaserData;
    rasp.startLaser();
    
    pause(1); % Wait for system to enter steady-state
    
    figure();
    
    function processLaserData(~, evnt)
        r_img = RangeImage(evnt.ranges);
        r_img.plot();
    end
end