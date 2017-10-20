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
        block = ShapeGen.rect(0.038,0.127);
        wm = WorldMap(rob, bounds);
            % Create a Ring of Blocks around Origin.
            r = 0.75;
            n = 6;
            ths = (0 : pi/2/n : pi/2 + pi/2/n); % Go one block beyond first quadrant
            for th = ths
                obs = wm.addObstacle(block);
                obs.pose = [r*cos(th) r*sin(th) th];
            end
        wm.createMap();
    end
    
    %% INITIALIZE
    fig = figure();
    rob.core.laser.NewMessageFcn = @processLaserData;
    rob.core.startLaser();
    
    pause(1); % Wait for system to enter steady-state
    
    while(1)
        rob.moveAt(0.1,0.2);
    end
    
    pause
    
    function processLaserData(~, evnt)
        r_img = RangeImage(evnt.Ranges);
        figure(fig);
        r_img.plot();
    end
end