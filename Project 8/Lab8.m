function Lab8(robot_id)
     mrpl = mrplSystem(robot_id, pose(0,0,0));
     
     mrpl.plottingOn(); % Turn on Plotting
     
     pause(5); % Wait for Lidar to init.
     
     mrpl.getNearestLineObject.poseVec
     mrpl.goTo_Rel( mrpl.getNearestLineObject() );
     
     mrpl.rob.core.forksUp();
     
     pause % Avoid GC
end