% Runs the Challenge Task of Lab4 where robot_id is the id of the RaspBot
% and type is a binary where 0 -> feedforward operation, 1 -> ffwd+fbk trim
function Lab4_Challenge(robot_id, type)
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
    
    delay = (3.239-2.862); % s, time delay between command signal and robot motion
    
    d_range = 0.01;     % m, distance away from target to be considered within range.
    t_over = 1;         % s, time for algorithm to run after reaching target
    
    %% PLOT SETUP
    fig = figure();
    axis equal
    legend('Reference Velocity', 'Reference Distance', 'Delayed Velocity', 'Delayed Distance', 'Measured Distance');
    %% ALGORITHM
    %rob.enablePositionPlotting();
    rob.waitForReady();
    
    %Anonymous function handle for velocity profile:
    uref = @(t)u_ref_ch(t,a_max,v_max,targ_dist);
    usig = @(t)uref(t); % Control Signal
    
    ts = [0]; % CPU time of each Reference Calculation
    us = [0]; % Reference Velocities
    ss = [0]; % Reference Distances
    u_delays = [0]; % Reference Velocities
    s_delays = [0]; % Reference Distances
    ds = [0]; % Robot Distance Measurements
    
    done = 0;
    t0 = tic;
    while(~done)
        ts(end+1) = toc(t0);
        us(end+1) = usig(ts(end));
        ss(end+1) = ss(end) + (us(end)+us(end-1))*(ts(end)-ts(end-1))/2;
        u_delays(end+1) = usig(ts(end)-delay);
        s_delays(end+1) = s_delays(end) + (u_delays(end)+u_delays(end-1))*(ts(end)-ts(end-1))/2;
        ds(end+1) = rob.hist_estPose(end).X;
        
        rob.moveAt(us(end), 0);
        
        figure(fig);
        clf(fig);
        hold on
            plot(ts,us,'r', ts,ss,'g', ts,u_delays,'y', ts,s_delays,'b', ts,ds,'k');
        hold off
        
        if within(s_delays(end), d_range, targ_dist)
            done = 1;
            rob.moveAt(0,0);
        end
        
        pause(0.01); % CPU Relief
    end % while ~done
    rob.moveAt(0,0);

end % #Lab4_Challenge