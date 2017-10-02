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
    fig_es = figure();
    pl_exs = PersistentPlot(fig_es, 0,0);
    pl_eys = PersistentPlot(fig_es, 0,0);
    pl_eths = PersistentPlot(fig_es, 0,0);
    title('Transient Errors')
    legend('Alongtrack (\delta x)','Crosstrack (\delta y)','Heading (\delta \theta)')
    axis equal
    
    tf = Trajectory_Follower(rob,ttc);
    tf.pid_controller.correctiveTime = 0.03;    % s, PID Time Constant
    
    rps = zeros(1,3);     % Vector of Robot Position Vectors across time [[Xr,Yr,th_r]]
    pps = zeros(1,3);     % Vector of Reference Position Vectors across time [[Xr,Yr,th_r]]
%     
%     r_xs = [];      % Robot X-Positions
%     r_ys = [];      % Robot Y-Positions
%     
%     p_xs = [];      % Trajectory X-Positions
%     p_ys = [];      % Trajectory Y-Positions
%     
    es = zeros(1,3);      % Vector of Robot Error Vectors across time [[ex,ey,eth]]
    
    ts = 0;        % Vector of Times of Execution
    
    first_loop = 1;
    clk = nan;
    T = 0;
    while(T < 12)%(ttc.times(end)+3))
        if(first_loop)
            clk = Clock();
        first_loop = 0;
        end
        
        T = clk.time();
        tf.follow_update(T);
        
        'fu'
        
        ts(end+1) = T;
        
        rps(end+1,:) = rob.hist_estPose(end).poseVec';
        pps(end+1,:) = ttc.getPose(T).poseVec';
        
        es(end+1,:) = tf.pid_controller.error_poses(end).poseVec';
        
        pause(0.01); % CPU Relief
    end
    rob.moveAt(0,0);
    rob.core.stop();
    
%     pl_exs.replaceXY(ts, es(:,1));
%     pl_eys.replaceXY(ts, es(:,2));
%     pl_eths.replaceXY(ts, es(:,3));
    
    fig_trajT = figure();
    hold on
        plot(ts, rps(:,1));
        plot(ts, rps(:,2));
        plot(ts, rps(:,3));
        plot(ts, pps(:,1));
        plot(ts, pps(:,2));
        plot(ts, pps(:,3));
    hold off
        
    fig_traj = figure();
    hold on
        plot(-rps(:,2), rps(:,1));
        plot(-pps(:,2), pps(:,1));
    hold off
    
    
end % #Lab5_Connor