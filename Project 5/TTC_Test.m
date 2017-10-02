function TTC_Test(robot_id)
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
    
    s_f = 1; 
    
    k_th = 2*pi/s_f;
    k_k = 15.1084;
    k_s = 1;
    k_v = 0.4;
    k_t = k_s/k_v;
    
    t_f = s_f/v_max + v_max / a_max;
    if(v_max^2/a_max > s_f) % not enough time to reach v_max and come back 
                             % down before reaching dist.
        v_p = sqrt(a_max*s_f); % Peak attainable velocity
        t_ramp = v_p/a_max;
        t_f = 2*t_ramp;
    end
    T_f = k_t * t_f;
    
    %% TRAJECTORY PLANNING
%     v_circ = @(~,~)0.2;
%     om_circ = @(~,~)0.2/0.1;
%     t_circ = (2*pi*0.1)/0.2;

    uref = @(t)u_ref_ch(t,a_max,v_max,s_f); % Model Velocity Curve (in Model Time, t)

    v8 = @(~,T)k_v*uref(T/k_t); % Takes Control Time, T
    
    % Omega, takes Control Time
    function om = om8(obj,T)
        V_t = @(TT)obj.V_func(obj,TT); % Control Velocity
        
        Ts = (obj.times(1) : obj.resolution : T);% Control Time
        %Control Path Length:
        S_t = trapz(Ts, arrayfun(V_t, Ts), 2); % Numeric Integration of Scalar Func.
        
        K = (k_k/k_s) * sin(k_th*S_t/k_s);
        om = K * obj.V_func(obj,T);
    end

    ttc = Trajectory_TimeCurve(v8,@om8, 0,T_f, 1000); % In Control Time
    
    %% ALGORITHM
    fig = figure();
    pl = PersistentPlot(fig, 0,0);
    axis equal
    
    first_loop = 1;
    clk = nan;
    T = 0;

    init_x = rob.hist_estPose(end).X;
    init_y = rob.hist_estPose(end).Y;
    xs = [];
    ys = [];
    while(T < T_f)
        if(first_loop)
            clk = Clock();
        first_loop = 0;
        end
        
        T = clk.time();
        X = ttc.getPose(T);
        V = ttc.getV(T);
        om = ttc.getOmega(T);
        rob.moveAt(V,om);
        
        xs(end+1) = rob.hist_estPose(end).X-init_x;
        ys(end+1) = rob.hist_estPose(end).Y-init_y;
        
        pl.addXY(-X.y, X.x);
        
        pause(0.01); % CPU Relief
    end
    rob.moveAt(0,0);
    rob.core.stop();
    figure(fig);
    hold on
        plot(-ys,xs);
    hold off
end