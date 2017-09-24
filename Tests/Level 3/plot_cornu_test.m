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
    bounds = 2 * ShapeGen.translatePts( ShapeGen.rect(1,1), 0,0.5 ); %[0.5 0; 0.5 1; -0.5 1; -0.5 0]; % Inverted U-Shaped Container
    person_lines = ShapeGen.rect(0.1,0.05);
    wm = WorldMap(rob, bounds);
        person = wm.addObstacle(person_lines); % Person to Follow.
        person.pose = [0.25 0.5 0]; % Move to Center Screen
    wm.createMap();
    pause(1) % Wait for World to Initialize
    
    %% PLOT
    V = 0.05; % m/s, Velocity
    k_w = 1/8;
    T_f = sqrt(32*pi);
    
    fig = figure();
    hold on
        pl_comm = plot(0, 0, 'k'); % Plot of Commanded Dead-Reckoning Positions ( int dX.(X,u) )
        pl_est = plot(0, 0, 'b');  % Plot of Measured/Estimated Dead-Reckoning Positions ( int dX.(X,z) )
    hold off
    axis([-2*0.125 0.5*0.125 -0.5*0.125 2*0.125]);
    legend('Position Reckoned from Commands', 'Position Estimated from Readings');
    
    st = tic;
    
    while(~is_done(toc(st)))
        pause(0.03); % CPU Relief
        
        rob.moveAt(V, k_w*toc(st));
        
        figure(fig);
        hold on
            set(pl_comm, 'xdata',-[rob.hist_commPose(:).Y], 'ydata',[rob.hist_commPose(:).X]);
            set(pl_est, 'xdata',-[rob.hist_estPose(:).Y], 'ydata',[rob.hist_estPose(:).X]);
        hold off
        refreshdata
        
        if(is_done(toc(st)))
            rob.moveAt(0,0);
        end
    end
    rob.moveAt(0,0);
    
    figure(fig);
    hold on
        set(pl_comm, 'xdata',-[rob.hist_commPose(:).Y], 'ydata',[rob.hist_commPose(:).X]);
        set(pl_est, 'xdata',-[rob.hist_estPose(:).Y], 'ydata',[rob.hist_estPose(:).X]);
    hold off
    refreshdata
    
    % Determines whether the motion is done based on the given starting
    % time.s
    function isdn = is_done(start_time)
        isdn = (start_time > T_f);
    end
end