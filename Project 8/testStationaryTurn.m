function testStationaryTurn(robot_id)
    mrpl = mrplSystem(robot_id, pose(0,0,0)); 
    mrpl.rob.core.stopLaser();
    mrpl.plottingOn(); % Turn on Plotting
    
    mrpl.turn_stationary(-pi/2);
    pause(0.5);
    %mrpl.turn_stationary(pi/2);
    %mrpl.goTo_Rel( pose(0.4,0.1,0.0) );
    
end