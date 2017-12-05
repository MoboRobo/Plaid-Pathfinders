function Lab12(robot_id, spd)
    units(); % Import Units
    global ft
    
    speed = spd;
    bounds = 4*ft*[1 0; 0 0; 0 -1; 1 -1]; % 4 Meter U with bottom left corner at origin
    wm = WorldMap(bounds);
    
    robot_starting_pose = pose(0.75*ft, -0.75*ft, -pi);
    
    ri = struct('mrpl',mrplSystem.empty);
    ri.mrpl = mrplSystem(robot_id, robot_starting_pose, wm);
   % ri = RobotInterface(robot_id, robot_starting_pose, wm);
    
    JS = JobScheduler(ri.mrpl);
    schedule = [...
        Job( JID.PICK, pose(3.5*ft, -1*ft, 0), speed ), ...
        Job( JID.DROP, pose(0.5*ft+0.06, -1.75*ft, pi), speed ), ...
        ...
        Job( JID.WAIT, 0.1 ), ...
        Job( JID.PICK, pose(3.5*ft, -2*ft, 0), 0.85*speed),  ...
        Job( JID.DROP, pose(0.5*ft+0.06, -2.25*ft, pi), speed), ...
        Job(JID.WAIT, 0.1),...
        Job( JID.PICK, pose(3.5*ft, -3*ft, 0), speed),  ...
        Job( JID.DROP, pose(0.5*ft+0.06, -2.75*ft, pi), speed), ...
        Job(JID.WAIT, 1)...
    ];
    JS.schedule(schedule);
    
    pause(3); % Ensure Robot Has Stable Localization before Beginnning
    
    while ( ~JS.jobs.isEmpty())
        JS.executeNext();
        pause(1);
    end
%      ri.mrpl.goTo( pose(-0.3048, -0.3048, 0.0) );
%      ri.mrpl.goTo( pose(-0.9144, -0.9144, -pi/2.0) );
%      ri.mrpl.goTo( pose(-0.6096, -0.6096, 0.0) );
end % #Lab12