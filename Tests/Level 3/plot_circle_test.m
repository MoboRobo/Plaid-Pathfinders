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
    rho = 0.1; % m, Radius of Curvature
    fig = figure();
    hold on
        pl_comm = plot(0, 0, 'k'); % Plot of Commanded Dead-Reckoning Positions ( int dX.(X,u) )
        pl_est = plot(0, 0, 'b');  % Plot of Measured/Estimated Dead-Reckoning Positions ( int dX.(X,z) )
    hold off
    axis([-2.5*rho 0.5*rho -1.5*rho 1.5*rho]);
    legend('Position Reckoned from Commands', 'Position Estimated from Readings');
    
    %st = tic;
    
    while(1)
        rob.moveAt(V, V/rho);
        
        % Commanded Position:
        cx = rob.hist_commPose(end).X;
        cy = rob.hist_commPose(end).Y;
        
        ex = rob.hist_estPose(end).X;
        ey = rob.hist_estPose(end).Y;

        figure(fig);
        hold on
            set(pl_comm, 'xdata',[get(pl_comm, 'xdata'), -cy], 'ydata',[get(pl_comm, 'ydata'), cx]);
            set(pl_est, 'xdata',[get(pl_est, 'xdata'), -ey], 'ydata',[get(pl_est, 'ydata'), ex]);
        hold off
        refreshdata
        
        pause(0.05);
    end
end