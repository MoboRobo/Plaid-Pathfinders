function Lab5_Connor(robot_id, fbktrim)
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
    v_max = 0.25;       % m/s, peak velocity of ffwd reference trajectory
    a_max = 3*0.25;     % m/s^2, peak acceleration of ffwd ref trajectory
    targ_dist = 1;      % m, target distance of the ffwd ref. trajectory
    s_f = targ_dist;
    t_ramp = v_max / a_max;
    t_f = s_f/v_max + t_ramp;
    %BUT:
    if(v_max^2/a_max > s_f) % not enough time to reach v_max and come back 
                             % down before reaching dist.
        v_p = sqrt(a_max*s_f); % Peak attainable velocity
        t_ramp = v_p/a_max;
        t_f = 2*t_ramp;
    end
    
    uref_linear = @(~,t)u_ref_ch(t,a_max,v_max,targ_dist);
    
    ttc = TTC_Figure8();%Trajectory_TimeCurve(uref_linear,@(~,~)0, 0,t_f, 500);
    
    %% ALGORITHM:
    fig_es = figure();
    pl_exs = PersistentPlot(fig_es, 0,0);
    pl_eys = PersistentPlot(fig_es, 0,0);
    pl_eds = PersistentPlot(fig_es, 0,0);
    pl_eths = PersistentPlot(fig_es, 0,0);
    title('Transient Errors');
    xlabel('Time [s]');
    ylabel('Position [m]');
    legend('Alongtrack (\deltax)','Crosstrack (\deltay)', 'Position (\deltas)', 'Heading (\delta\theta)');
    axis equal
    
    tf = Trajectory_Follower(rob,ttc);
        tf.fbk_trim = fbktrim;
        tf.pid_controller.correctiveTime = 1.8*ttc.times(end);    % s, PID Time Constant
        tf.pid_controller.k_p = 1;
        tf.pid_controller.k_d = 0;
        tf.pid_controller.k_i = 0.0;
    
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
    while(T < (ttc.times(end)+ttc.send_delay+1)) % Run for one second beyond end of reference trajectory
        if(first_loop)
            clk = Clock();
        first_loop = 0;
        end
        
        T = clk.time();
        tf.follow_update(T);
        
%         pl_exs.addXY(ts, es(end,1));
%         pl_eys.addXY(ts, es(end,2));
%         pl_eds.addXY(ts, sqrt(es(end,1).^2 + es(end,2).^2));
%         pl_eths.addXY(ts, es(end,3));
        
        ts(end+1) = T;
        
        rps(end+1,:) = rob.hist_estPose(end).poseVec';
        pps(end+1,:) = ttc.getPose(T).poseVec';
        
        es(end+1,:) = tf.pid_controller.error_poses(end).poseVec';
        
        pause(0.01); % CPU Relief
    end
    rob.moveAt(0,0);
    rob.core.stop();
    
    pl_exs.replaceXY(ts, es(:,1));
    pl_eys.replaceXY(ts, es(:,2));
    pl_eds.replaceXY(ts, sqrt(es(:,1).^2 + es(:,2).^2));
    pl_eths.replaceXY(ts, es(:,3));
    
    fig_trajT = figure();
    hold on
        plot(ts, rps(:,1));
        plot(ts, rps(:,2));
        plot(ts, rps(:,3));
        plot(ts, pps(:,1));
        plot(ts, pps(:,2));
        plot(ts, pps(:,3));
    hold off
    xlabel('Time [s]');
    ylabel('Position [m]');
    legend('Robot X','Robot Y','Robot \theta', 'Reference X','Reference Y','Reference \theta');
        
    fig_traj = figure();
    hold on
        plot(-rps(:,2), rps(:,1));
        plot(-pps(:,2), pps(:,1));
    hold off
    xlabel('Position [m]');
    ylabel('Position [m]');
    legend('Robot Trajectory','Reference Trajectory');
    
    
end % #Lab5_Connor