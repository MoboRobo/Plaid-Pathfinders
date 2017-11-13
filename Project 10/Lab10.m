function Lab10(robot_id, robot_starting_pose)
    bounds = 2*[-1 0; 0 0; 0 -1]; % 2 Meter L intersecting origin
    wm = WorldMap(bounds);

    ri = RobotInterface(robot_id, robot_starting_pose, wm);
    
    ri.run();
    
end % #Lab10