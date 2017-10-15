function Lab6(robot_id, scale, fbktrim)
    global clk rob
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
    k_p = 0.0;
    k_d = 0.0;
    k_i = 0.0;
    k_tau = 1.4;
    
    rtA = Trajectory_CubicSpiral.planTrajectory(0.3048,0.3048,0.0, 1, 201,scale);%Trajectory_TimeCurve(uref_linear,@(~,~)0, 0,t_f, 500);
    rtB = Trajectory_CubicSpiral.planTrajectory(-0.6096,-0.6096,-pi/2.0, 1, 201,scale);
    rtC = Trajectory_CubicSpiral.planTrajectory(-0.3048,0.3048,pi/2.0, 1, 201,scale);
    
    %% ALGORITHM:
    es = zeros(1,3);      % Vector of Robot Error Vectors across time [[ex,ey,eth]]
    
    ts = 0;               % Vector of Times of Execution
    
    tfA = Trajectory_Follower(rob,rtA);
        tfA.fbk_trim = fbktrim;
        tfA.fbk_controller.correctiveTime = k_tau*rtA.getFinalTime();    % s, PID Time Constant
        tfA.fbk_controller.k_p = k_p;
        tfA.fbk_controller.k_d = k_d;
        tfA.fbk_controller.k_i = k_i;
    tfB = Trajectory_Follower(rob,rtB);
        tfB.fbk_trim = fbktrim;
        tfB.fbk_controller.correctiveTime = k_tau*rtB.getFinalTime();    % s, PID Time Constant
        tfB.fbk_controller.k_p = k_p;
        tfB.fbk_controller.k_d = k_d;
        tfB.fbk_controller.k_i = k_i;
    tfC = Trajectory_Follower(rob,rtC);
        tfC.fbk_trim = fbktrim;
        tfC.fbk_controller.correctiveTime = k_tau*rtC.getFinalTime();    % s, PID Time Constant
        tfC.fbk_controller.k_p = k_p;
        tfC.fbk_controller.k_d = k_d;
        tfC.fbk_controller.k_i = k_i;
    
    run_trajectory(tfA);
     pause(0.3); % short pause between each
    run_trajectory(tfB);
     pause(0.3); % short pause between each
    run_trajectory(tfC);

    rob.moveAt(0,0);
    rob.core.stop();
    
    
end % #Lab6

% Runs the Robot along the Trajectory Specified by the Given
% TrajectoryFollower
function run_trajectory(tf)
    global clk rob
    rob.resetStateData(); % Reset Robot Odometry
    
    x = [tf.fbk_controller.k_p, tf.fbk_controller.k_d, tf.fbk_controller.k_i]
    
    rxs = zeros(1000,0); % History of Robot X Positions
    rys = zeros(1000,0); % History of Robot Y Positions
    
    txs = zeros(1000,0); % History of Trajectory X Positions
    tys = zeros(1000,0); % History of Trajectory Y Positions

    Et = Inf;
    E_head = Inf;
    pf = tf.rt.getFinalPose();
    
    count = 1;
    S0 = 0;
    first_loop = 1;
    clk = nan;
    S = 0;
    S_f = tf.rt.getFinalDist();
    T = 0;
    %while(~within(Et,0.02,0))
    while (T < tf.rt.getFinalTime()+1 )%&& (~within(E_head,0.01,0) || T < 0.9*tf.rt.getFinalTime()))
        if(first_loop)
            clk = Clock();
            S0 = rob.measTraj.s_f-0.0001;
        first_loop = 0;
        end
        
        T = clk.time();
        S = rob.measTraj.s_f - S0;
        
        tf.follow_update_t(T);
        
        rp = rob.measTraj.p_f;
        rxs(count) = rp.X;
        rys(count) = rp.Y;
        
        tp = tf.rt.p_t(T);
        txs(count) = tp.X;
        tys(count) = tp.Y;
        
        Et = sqrt( (pf.X-rp.X)^2 + (pf.Y-rp.Y)^2 );
        %E_head = (pf.X-rp.X)*sin(rp.th) - sign(rp.th)*(pf.Y-rp.Y)*cos(rp.th);
        
        count = count+1;
        pause(0.01); % CPU Relief
    end
    
    rob.moveAt(0,0);
    
    rp = rob.measTraj.p_f;
    rxs(count) = rp.X;
    rys(count) = rp.Y;
    
    pf = tf.rt.getFinalPose();
    figure();
    title({strcat('Trajectory to : ', num2str(pf.X),',',num2str(pf.Y)),...
           strcat('Terminal Odometry Error: ', num2str(Et), 'm')});
    xlabel('World X-Position Relative to Start [m]');
    ylabel('World Y-Position Relative to Start [m]');
    hold on
        plot(rxs,rys);
        plot(txs,tys);
    hold off
    legend('Robot Position', 'Trajectory Position');
    axis equal
end