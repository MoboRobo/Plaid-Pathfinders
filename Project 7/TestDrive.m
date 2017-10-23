function TestDrive(robot_id)
%     mrplt = mrplTestbox(robot_id);
%     mrplt.test();
     mrpl = mrplSystem(robot_id, pose(0,0,0));
     
     mrpl.plottingOn(); % Turn on Plotting
     %Trajectory.plot_thetas(1); % Turn on Heading Plotting
     mrpl.debugging.error_plots = 1;
     mrpl.debugging.comm_plots = 1;
     
     mrpl.goTo_Rel( pose(0.4,0.1,0.0) );
     pause(0.3); % short pause between each
     mrpl.goTo_Rel( pose(0.4,0.1,0.0) );
     pause(0.3); % short pause between each
     mrpl.goTo_Rel( pose(0.4,0.1,0.0) );
     pause(0.3); % short pause between each
     
     pause % Avoid GC
end