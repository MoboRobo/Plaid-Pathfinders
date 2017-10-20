function Lab8_TestBench(robot_id)
    %% SETUP ROBOT
    rasp = raspbot(robot_id, [0; 0; pi/2]);
    rob = P2_Robot(rasp);
    if(~strcmp(robot_id,'sim'))
        rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
        RangeImage.INDEX_OFFSET(5);
    end
    
    %% SETUP MAPPING
    if(strcmp(robot_id,'sim'))
        bounds = 2.5*[0.5 0; 0.5 1; -0.5 1; -0.5 0]; % Inverted U-Shaped Container
        person_lines = ShapeGen.rect(0.1,0.05);
        wm = WorldMap(rob, bounds);
            person = wm.addObstacle(person_lines); % Person to Follow.
            person.pose = [0.25 0.5 0]; % Move to Center Screen
        wm.createMap();
    end
    
    %% INITIALIZE
    fig = figure();
    rob.core.laser.NewMessageFcn = @processLaserData;
    rob.core.startLaser();
    
    pause(1); % Wait for system to enter steady-state
    
    function processLaserData(~, evnt)
        r_img = RangeImage(evnt.Ranges);
        figure(fig);
        r_img.plot();
    end
end