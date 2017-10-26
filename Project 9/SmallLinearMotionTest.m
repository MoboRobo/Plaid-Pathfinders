function SmallLinearMotionTest(robot_id)
     mrpl = mrplSystem(robot_id, pose(0,0,0));
     
     mrpl.plottingOn(); % Turn on Plotting
     
     pause(2); % Wait for Robot to Initialize
     
     mrpl.goTo_X_Small(0.05);
     
     pause % Avoid GC
end