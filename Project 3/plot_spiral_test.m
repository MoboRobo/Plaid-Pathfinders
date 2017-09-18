function plot_spiral_test(robot_id)
%% SETUP ROBOT
    rasp = raspbot(robot_id, [0; 0; pi/2])
    rob = P2_Robot(rasp);
    if(~strcmp(robot_id,'sim'))
        rob_type = 'raspbot';
        rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
        RangeImage.INDEX_OFFSET(5);
        rob.core.forksDown(); % Prevent Brown-out
    end

    %% SETUP MAPPING
    bounds = 3.5*[0.5 0; 0.5 1; -0.5 1; -0.5 0]; % Inverted U-Shaped Container
    person_lines = ShapeGen.rect(0.1,0.05);
    wm = WorldMap(rob, bounds);
        person = wm.addObstacle(person_lines); % Person to Follow.
        person.pose = [0.25 0.5 0]; % Move to Center Screen
    wm.createMap();
    pause(1) % Wait for World to Initialize
    
    %% PLOT
    rob.logEncoders();
    figure;
    p = plot(-rob.hist_y,rob.hist_x);
    t0 = tic
    while(toc(t0) < sqrt(32*pi))
        rob.moveAt(0.1, toc(t0)/8);
        set(p, 'xdata', [get(p,'xdata') -rob.hist_y(end)], 'ydata', [get(p,'ydata') rob.hist_x(end)]);
        refreshdata
        pause(0.05);
    end
    rob.moveAt(0,0)
end