function testObscure(robot_id)
    bounds = 2*[-1 0; 0 0; 0 -1]; % 2 Meter L intersecting origin
    wm = WorldMap(bounds);
    
     mrpl = mrplSystem(robot_id, pose(0,0,0), wm);
     rangeImg = mrpl.rob.histLaser.last;
     
     isObs = isObscured(rangeImg);
     disp(isObs);
     
end