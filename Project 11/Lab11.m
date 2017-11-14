function Lab11(robot_id)
    bounds = 2*[-1 0; 0 0; 0 -1]; % 2 Meter L intersecting origin
    wm = WorldMap(bounds);
    
    robot_starting_pose = pose(-0.6096, -0.6096, -pi/2);
    
    ri = struct('mrpl',mrplSystem.empty);
    ri.mrpl = mrplSystem(robot_id, robot_starting_pose, wm);
    
    ri.mrpl.plottingOn(); % Turn on Plotting
     %Trajectory.plot_thetas(1); % Turn on Heading Plotting
%      ri.mrpl.debugging.error_plots = 1;
%      ri.mrpl.debugging.comm_plots = 1;

    pause(3); % Ensure Robot Has Stable Localization before Beginnning
     
%      ri.mrpl.goTo( pose(-0.3048, -0.3048, 0.0) );
%      ri.mrpl.goTo( pose(-0.9144, -0.9144, -pi/2.0) );
%      ri.mrpl.goTo( pose(-0.6096, -0.6096, 0.0) );
     
     ri.mrpl.goTo( addPoses(robot_starting_pose, pose(0.3048,0.3048,0.0)) );
%      ri.mrpl.turn_stationary(pi);
     pause(2);
     ri.mrpl.goTo( addPoses(robot_starting_pose, pose(-0.3048,-0.3048,-pi/2.0)) );
%      ri.mrpl.turn_stationary(pi);
     pause(2);
     ri.mrpl.goTo( robot_starting_pose );
     
%      ri.mrpl.goTo_Rel( pose(0.3048,0.3048,0.0) );
%      ri.mrpl.goTo_Rel( pose(-0.6096,-0.6096,-pi/2.0) );
%      ri.mrpl.goTo_Rel( pose(-0.3048,0.3048,pi/2.0) );
     
     pause % Avoid GC
end