function plot_circle_test(robot_id)
    %% SETUP ROBOT
    rasp = raspbot(robot_id, [0; 0; pi/2])
    rob = P2_Robot(rasp);
    if(~strcmp(robot_id,'sim'))
        rob_type = 'raspbot';
        rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
        RangeImage.INDEX_OFFSET(5);
        rob.core.forksDown(); % Prevent Brown-out
    end
    %% SETUP SIM MAPPING
    bounds = 2 * ShapeGen.translatePts( ShapeGen.rect(1,1), 0,0.5 ); %[0.5 0; 0.5 1; -0.5 1; -0.5 0]; % Inverted U-Shaped Container
    person_lines = ShapeGen.rect(0.1,0.05);
    wm = WorldMap(rob, bounds);
        person = wm.addObstacle(person_lines); % Person to Follow.
        person.pose = [0.25 0.5 0]; % Move to Center Screen
    wm.createMap();
    pause(1) % Wait for World to Initialize
    
    %% TASK PARAMETERS
    V = 0.2;
    s_f = 1;
    t_f = s_f/V;
    
    k_th = 2*pi/s_f;
    k_k = 15.1084;
    k_s = 3;
    
    T_f = k_s*t_f;
    
    %% ALGORITHM
    rob.enablePositionPlotting();
    
    rob.waitForReady();
    
    tt = 0; % Current Time
    ST = tic;
    while(~is_done(toc(ST)))
        tt = toc(ST);
        
        rob.moveAt(V, (k_k/k_s)*sin(k_th*V*tt/k_s)*V);

        if(is_done(tt))
            rob.moveAt(0,0);
        end
        
        pause(0.01); % CPU Relief
    end
    rob.moveAt(0,0);
    
    % Determines whether the motion is done based on the given starting
    % time.s
    function isdn = is_done(start_time)
        isdn = (start_time > T_f);
    end
end