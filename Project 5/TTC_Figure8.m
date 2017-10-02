%% Figure-8 Trajectory Time Curve
% Returns the TTC for implementing a figure 8, for which the user can
% optionally supply custom k_s, k_v, and s_f parameters.
function ttc = TTC_Figure8(ks,kv,sf)
    v_max = 0.25;       % m/s, peak velocity of ffwd reference trajectory
    a_max = 3*0.25;     % m/s^2, peak acceleration of ffwd ref trajectory
    
    %% TASK_PARAMETERS
    k_s = 3;
    k_v = 1;  
    s_f = 1;
    if(nargin > 2)
        k_s = ks; k_v = kv; s_f = sf;
    elseif(nargin > 1)
        k_s = ks; k_v = kv;
    elseif(nargin > 0)
        k_s = ks;
    end % nargin>n?
    
    k_th = 2*pi/s_f;
    k_k = 15.1084;
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

    % Trapezoidal Reference Velocity:
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
end % #TTC_Figure8