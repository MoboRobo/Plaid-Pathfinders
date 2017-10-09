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
    k_p = 1.0;
    k_d = 0.0;
    k_i = 0.0;
    
    rtA = Trajectory_CubicSpiral.planTrajectory(0.3048,0.3048,0.0, 1, 201,scale);%Trajectory_TimeCurve(uref_linear,@(~,~)0, 0,t_f, 500);
    rtB = Trajectory_CubicSpiral.planTrajectory(-0.6096,-0.6096,-pi/2.0, 1, 201,scale);
    rtC = Trajectory_CubicSpiral.planTrajectory(-0.3048,0.3048,pi/2.0, 1, 201,scale);
    
    %% ALGORITHM:
    es = zeros(1,3);      % Vector of Robot Error Vectors across time [[ex,ey,eth]]
    
    ts = 0;        % Vector of Times of Execution
    
    tfA = Trajectory_Follower(rob,rtA);
        tfA.fbk_trim = fbktrim;
        tfA.pid_controller.correctiveTime = 1.8*rtA.getFinalTime();    % s, PID Time Constant
        tfA.pid_controller.k_p = k_p;
        tfA.pid_controller.k_d = k_d;
        tfA.pid_controller.k_i = k_i;
    tfB = Trajectory_Follower(rob,rtB);
        tfB.fbk_trim = fbktrim;
        tfB.pid_controller.correctiveTime = 1.8*rtB.getFinalTime();    % s, PID Time Constant
        tfB.pid_controller.k_p = k_p;
        tfB.pid_controller.k_d = k_d;
        tfB.pid_controller.k_i = k_i;
    tfC = Trajectory_Follower(rob,rtC);
        tfC.fbk_trim = fbktrim;
        tfC.pid_controller.correctiveTime = 1.8*rtC.getFinalTime();    % s, PID Time Constant
        tfC.pid_controller.k_p = k_p;
        tfC.pid_controller.k_d = k_i;
        tfC.pid_controller.k_i = k_d;
    
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
    
    rxs = zeros(1000,0); % History of Robot X Positions
    rys = zeros(1000,0); % History of Robot Y Positions
    
    txs = zeros(1000,0); % History of Trajectory X Positions
    tys = zeros(1000,0); % History of Trajectory Y Positions
    
    p0 = rob.hist_estPose(end);
    x0 = p0.X; y0 = p0.Y;
    
    count = 0;
    S0 = 0;
    first_loop = 1;
    clk = nan;
    S = 0;
    S_f = tf.rt.getFinalDist();
    while(~within(S,0.005,S_f))
        if(first_loop)
            clk = Clock();
            S0 = rob.hist_estDist(end)-0.0001;
        first_loop = 0;
        end
        
        T = clk.time();
        S = rob.hist_estDist(end) - S0;
        t_s = tf.rt.t_s(S);
        
        tf.follow_update_s(S);
        
        rp = rob.hist_estPose(end);
        rxs(count) = rp.X - x0;
        rys(count) = rp.Y - y0;
        
        tp = tf.rt.p_s(S);
        txs(count) = tp.X;
        tys(count) = tp.Y;
        
        count = count+1;
        pause(0.01); % CPU Relief
    end
    
    rob.moveAt(0,0);
%     
%     pf = tf.rt.getFinalPose();
%     figure();
%     title(strcat('Trajectory to : ', num2str(pf.X),',',num2str(pf.Y)));
%     xlabel('World X-Position Relative to Start [m]');
%     ylabel('World Y-Position Relative to Start [m]');
%     hold on
%         plot(rxs,rys);
%         plot(txs,tys);
%     hold off
%     axis equal
end