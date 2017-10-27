function SmallStationaryTurnTest(robot_id)
     mrpl = mrplSystem(robot_id, pose(0,0,0));
     
     mrpl.plottingOn(); % Turn on Plotting
     
     pause(2); % Wait for Robot to Initialize
     
     mrpl.goTo_th_Small(pi/2);
     
     pause % Avoid GC
end