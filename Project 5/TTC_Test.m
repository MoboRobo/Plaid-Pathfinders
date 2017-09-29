function TTC_Test
    %% TASK PARAMETERS
    v = 0.2;
    s_f = 1;
    t_f = s_f/v;
    
    k_th = 2*pi/s_f;
    k_k = 15.1084;
    k_s = 3;
    k_v = 1;
    
    T_f = (k_s/k_v) * t_f;
    
    %% TRAJECTORY PLANNING
%     v_circ = @(~,~)0.2;
%     om_circ = @(~,~)0.2/0.1;
%     t_circ = (2*pi*0.1)/0.2;
    
    v8 = @(obj,t) k_v*v; %Anonymous function
    function om = om8(obj,t)
        om = (k_k/k_s) * sin(k_th*v*t*k_v/k_s) * obj.V_func(t);
    end

    ttc = Trajectory_TimeCurve(v8,@om8, 0,T_f, 1000);
    
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