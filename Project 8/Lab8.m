function Lab8(robot_id)
     mrpl = mrplSystem(robot_id, pose(0,0,0));
     
     mrpl.plottingOn(); % Turn on Plotting
     
     pause(2); % Wait for Robot to Initialize
     
     p_nlo_r = mrpl.getNearestLineObject();
     p_acq_r = mrpl.acquisitionPose(p_nlo_r, 0.067, 0.02, 0);
     mrpl.goTo_Rel( p_acq_r );
     
     mrpl.rob.core.forksUp();
     
     pause % Avoid GC
end