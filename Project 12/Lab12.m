function Lab12
    bounds = 2*[-1 0; 0 0; 0 -1]; % 2 Meter L intersecting origin
    wm = WorldMap(bounds);
    
    robot_starting_pose = pose(0.75*ft, -0.75*ft, -pi);
    
%     ri = struct('mrpl',mrplSystem.empty);
    ri = RobotInterface(robot_id, robot_starting_pose, wm);
    
    JS = JobScheduler(ri.mrpl);
    schedule = [...
        Job( JID.PICK, pose(3.5*ft, -1*ft, 0), 0.2 ), ...
        Job( JID.DROP, pose(0.75*ft, -1.75*ft, pi), 0.2 ), ...
        ...
        Job( JID.WAIT, 1 ) ...
    ];
    JS.schedule(schedule);
        
        
    
    pause(3); % Ensure Robot Has Stable Localization before Beginnning
     
%      ri.mrpl.goTo( pose(-0.3048, -0.3048, 0.0) );
%      ri.mrpl.goTo( pose(-0.9144, -0.9144, -pi/2.0) );
%      ri.mrpl.goTo( pose(-0.6096, -0.6096, 0.0) );
end % #Lab12