function TTC_Test
    global clk
    clk = Clock();
    
    %% TASK PARAMETERS
    v_max = 0.25;       % m/s, peak velocity of ffwd reference trajectory
    a_max = 3*0.25;     % m/s^2, peak acceleration of ffwd ref trajectory
    
    s_f = 1; 
    
    k_th = 2*pi/s_f;
    k_k = 15.1084;
    k_s = 3;
    k_v = 2;
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
    
    first_loop = 1;
    clk = nan;
    T = 0;
    while(T < T_f)
        if(first_loop)
            clk = Clock();
        first_loop = 0;
        end
        
        T = clk.time();
        X = ttc.getPose(T);
        
        pl.addXY(-X.y, X.x);
        
        pause(0.01); % CPU Relief
    end
end