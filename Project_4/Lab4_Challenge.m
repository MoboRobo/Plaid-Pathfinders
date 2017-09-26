% Runs the Challenge Task of Lab4 where robot_id is the id of the RaspBot
% and type is a binary where 0 -> feedforward operation, 1 -> ffwd+fbk trim
function Lab4_Challenge(robot_id, type)
global rob
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
    v_max = 0.25;       % m/s, peak velocity of ffwd reference trajectory
    a_max = 3*0.25;     % m/s^2, peak acceleration of ffwd ref trajectory
    targ_dist = 1;      % m, target distance of the ffwd ref. trajectory
    
    delay = 0.118;      % s, time delay between command signal and robot motion
    
    d_range = 1e-3;     % m, distance away from target to be considered within range.
    t_over = 1;         % s, time for algorithm to run after reaching target
    
    %% PLOT SETUP
    fig = figure();
    %u_plot = PersistentPlot(fig, 0,0, 'r');
    %s_plot = PersistentPlot(fig, 0,0, 'g');
    %u_delay_plot = PersistentPlot(fig, 0,0, 'm');
    s_delay_plot = PersistentPlot(fig, 0,0, 'k');
    d_plot = PersistentPlot(fig, 0,0, 'b'); % Measured Distance
    legend('Delayed Distance', 'Measured Distance');
    title('Robot Position');
    xlabel('CPU Time Elasped [s]');
    ylabel('Position [m]');
%     axis equal
    
    fig = figure();
    diff_plot = PersistentPlot(fig, 0,0, 'r');
    title('Position Error');
    xlabel('CPU Time Elasped [s]');
    ylabel('Position Error [m]');
    
    %% ALGORITHM
    %rob.enablePositionPlotting();
    rob.waitForReady();
    
    ts = [0]; % CPU time of each Reference Calculation
    u_refs = [0]; % Reference Velocities
    s_refs = [0]; % Reference Distances
    u_ref_delays = [0]; % Reference Velocities
    s_ref_delays = [0]; % Reference Distances
    ds = [0]; % Robot Distance Measurements
    
    errors = [0];
    
    
    %Anonymous function handle for velocity profile:
    uref = @(t)u_ref_ch(t,a_max,v_max,targ_dist);
    
    done = 0;
    terminating = 0;
    t_beginTermination = 0;
    d0 = rob.hist_estPose(end).X;
    t0 = tic;
    while(~done)
        upid = @(t)u_pid(t,s_ref_delays(end),ds(end));
        if(type)
            usig = @(t)uref(t)+upid(t); % Control Signal
        else
            usig = @(t)uref(t); % Control Signal
        end
        
        ts(end+1) = toc(t0);
%         u_refs(end+1) = uref(ts(end));
%         s_refs(end+1) = s_refs(end) + (u_refs(end)+u_refs(end-1))*(ts(end)-ts(end-1))/2;
        u_ref_delays(end+1) = uref(ts(end)-delay);
        s_ref_delays(end+1) = s_ref_delays(end) + (u_ref_delays(end)+u_ref_delays(end-1))*(ts(end)-ts(end-1))/2;
        ds(end+1) = rob.hist_estPose(end).X - d0;
        
        errors(end+1) = s_ref_delays(end) - ds(end);
        
        rob.moveAt(usig(ts(end)), 0);
        
%         u_plot.update_replaceXY(ts,us);
%         s_plot.update_replaceXY(ts,ss);
%         u_delay_plot.update_replaceXY(ts,u_delays);
%         s_delay_plot.update_replaceXY(ts,s_ref_delays);
%         d_plot.update_replaceXY(ts,ds);
%         
%         diff_plot.update_replaceXY(ts,errors);
        
        if within(s_ref_delays(end), d_range, targ_dist)
            if(~terminating)% Just entered (or re-entered target zone).
                terminating = 1;
                t_beginTermination = tic;
            end
        else
            terminating = 0; % re-zero if no longer in zone
        end
        if (terminating)
            if toc(t_beginTermination) > 1 % Must have stayed within zone 
                                           % for 1 second to terminate
                done = 1;
                rob.moveAt(0,0);
            end
        end
        
        pause(0.05); % CPU Relief
    end % while ~done
    rob.moveAt(0,0);
    
    s_delay_plot.update_replaceXY(ts,s_ref_delays);
    d_plot.update_replaceXY(ts,ds);

    diff_plot.update_replaceXY(ts,errors);

end % #Lab4_Challenge