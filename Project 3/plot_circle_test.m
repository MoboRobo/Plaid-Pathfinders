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
    
    %% SETUP MAPPING
    bounds = 3.5*[0.5 0; 0.5 1; -0.5 1; -0.5 0]; % Inverted U-Shaped Container
    person_lines = ShapeGen.rect(0.1,0.05);
    wm = WorldMap(rob, bounds);
        person = wm.addObstacle(person_lines); % Person to Follow.
        person.pose = [0.25 0.5 0]; % Move to Center Screen
    wm.createMap();
    pause(1) % Wait for World to Initialize
    
    %% PLOT
    fig = figure();
    pl = scatter([], []);
    axis([-3 3 -3 3])
    st = tic;
    while(1)
        rob.moveAt(0.1, 0.1/0.1);
        
        xx = rob.hist_commPose(end).X;
        yy = rob.hist_commPose(end).Y;
        
        figure(fig);
        set(pl, 'XData',[get(pl, 'XData') -yy], 'YData',[get(pl, 'YData') xx]);
        
        pause(0.05);
    end
end