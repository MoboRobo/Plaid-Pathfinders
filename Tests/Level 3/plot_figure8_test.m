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
    V = 0.2;
    s_f = 1;
    t_f = s_f/V;
    
    k_th = 2*pi/s_f;
    k_k = 15.1084;
    k_s = 3;
    
    T_f = k_s*t_f;
    
    fig = figure();
    hold on
        pl_comm = plot(0, 0, 'k-'); % Plot of Commanded Dead-Reckoning Positions ( int dX.(X,u) )
        pl_est = plot(0, 0, 'b');  % Plot of Measured/Estimated Dead-Reckoning Positions ( int dX.(X,z) )
    hold off
    axis equal
    legend('Position Reckoned from Commands', 'Position Estimated from Readings');
    drawnow
    
    pause(3); % WAIT FOR EVERYTHING TO INITIALIZE
    
    pl_period = 1/3; % Hz, Plotting Period (time between plots)
    pl_Tlast = 0; % s, Time of Last Plotting

    tt = 0; % Current Time
    ST = tic;
    while(~is_done(toc(ST)))
        tt = toc(ST);
        
        rob.moveAt(V, (k_k/k_s)*sin(k_th*V*tt/k_s)*V);
        
        if((tt-pl_Tlast) > pl_period)
         pl_Tlast = tt;
            figure(fig);
            hold on
                set(pl_comm, 'xdata',-[rob.hist_commPose(:).Y], 'ydata',[rob.hist_commPose(:).X]);
                set(pl_est, 'xdata',-[rob.hist_estPose(:).Y], 'ydata',[rob.hist_estPose(:).X]);
            hold off
        end % if toc(ST)-pl_Tlast > 1/pl_Hz

        if(is_done(tt))
            rob.moveAt(0,0);
        end
        
        pause(0.01); % CPU Relief
    end
    rob.moveAt(0,0);
    
    % Final Plot:
    figure(fig);
    hold on
        set(pl_comm, 'xdata',-[rob.hist_commPose(:).Y], 'ydata',[rob.hist_commPose(:).X]);
        set(pl_est, 'xdata',-[rob.hist_estPose(:).Y], 'ydata',[rob.hist_estPose(:).X]);
    hold off
    
    % Determines whether the motion is done based on the given starting
    % time.s
    function isdn = is_done(start_time)
        isdn = (start_time > T_f);
    end
end