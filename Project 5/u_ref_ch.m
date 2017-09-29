% Implementation of Lab 4's reference velocity trajectory curve for use in 
% the challenge task where:
% t is the time in the trajectory for which the control signal, u, is to be
% evaluated, such that 0<=t<=t_f
% a_max is the maximum acceleration of the trajectory
% v_max is the maximum velocity of the trajectory
% dist is the target distance to be traversed by the trajectory.
function u = u_ref_ch(t, a_max, v_max, dist, ~)
    %Magnitude of Distance to be travelled:
    s_f = abs(dist);
    %Duration of the constant acceleration, a_max, ramp up/down:
    t_ramp = v_max / a_max;
    %Total duration of the trajectory profile:
    t_f = s_f/v_max + t_ramp;
    %BUT:
    if(v_max^2/a_max > s_f) % not enough time to reach v_max and come back 
                             % down before reaching dist.
        v_p = sqrt(a_max*s_f); % Peak attainable velocity
        t_ramp = v_p/a_max;
        t_f = 2*t_ramp;
    end
    
    u = 0;
    % Determine Control Signal from Position in Velocity Trajectory:
    if(t < 0)
        u = 0;
    elseif(t <= t_ramp)
        u = a_max*t;
    elseif( (t_f-t) <= t_ramp && (t_f-t) > 0)
        u = a_max*(t_f-t);
    elseif(t_ramp<t && t<=(t_f-t_ramp))
        u = v_max;
    end
    
    if(dist < 0)
        u = -u;
    end
end % #u_ref_ch