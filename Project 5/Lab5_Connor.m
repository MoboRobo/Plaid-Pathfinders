function Lab5_Connor(robot_id)
    global clk
    %% SETUP ROBOT
    rasp = raspbot(robot_id, [0; 0; pi/2])
    rob = P2_Robot(rasp);
    if(~strcmp(robot_id,'sim'))
        rob_type = 'raspbot';
        rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
        RangeImage.INDEX_OFFSET(5);
        rob.core.forksDown(); % Prevent Brown-out
    end
    
    clk = Clock();
    
    %% TASK PARAMETERS
    ttc = TTC_Figure8();
    
    %% ALGORITHM:
    tf = Trajectory_Follower(rob,ttc);
    
    first_loop = 1;
    clk = nan;
    T = 0;
    while(T < ttc.times(end))
        if(first_loop)
            clk = Clock();
        first_loop = 0;
        end
        
        T = clk.time();
        tf.follow_update(T)
        
        pause(0.01); % CPU Relief
    end
    rob.moveAt(0,0);
    rob.core.stop();
end % #Lab5_Connor