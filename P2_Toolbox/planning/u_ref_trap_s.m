% DISTANCE PARAMETERIZED VELOCITY-TIME TRAPEZOIDAL PROFILE (trapezoid in
% V-t space).
% Implementation of Lab 4's reference velocity trajectory curve for use in 
% the challenge task where:
% s is the dist in the trajectory for which the control signal, u, is to be
% evaluated, such that 0<=t<=t_f
% a_max is the maximum acceleration of the trajectory
% v_max is the maximum velocity of the trajectory
% dist is the target distance to be traversed by the trajectory.
% give_Tfinal returns the Final Time of the Trajectory.
function ret = u_ref_trap_s(s, a_max, v_max, dist, ~, give_Tfinal)
    %Magnitude of Distance to be travelled:
    s_f = abs(dist);
    %Duration of the constant acceleration, a_max, ramp up/down:
    t_ramp = v_max / a_max;
    s_ramp = v_max^2 / 2 / a_max;
    %Total duration of the trajectory profile:
    t_f = s_f/v_max + t_ramp;
    %BUT:
    v_p = v_max; % Normal Condition
    if(v_max^2/a_max > s_f) % not enough time to reach v_max and come back 
                             % down before reaching dist.
        v_p = sqrt(a_max*s_f); % Peak attainable velocity
        t_ramp = v_p/a_max;
        t_f = 2*t_ramp;
    end
    
    u = 0;
    % Determine Control Signal from Position in Velocity Trajectory:
    if(s < 0)
        u = 0;
    elseif(s <= s_ramp) % Up ramp
        u = sqrt(2*a_max*s);
    elseif( (s_f-s) <= s_ramp && (s_f-s) > 0) % Down ramp
        u = sqrt(2*a_max*(s_f-s));
    elseif(s_ramp<s && s<=(s_f-s_ramp)) % plateau
        u = v_max;
    end
    
    if(dist < 0)
        u = -u;
    end
    
    if(give_Tfinal)
        ret = t_f;
    else
        ret = u;
    end % give_Tfinal?
end % #u_ref_ch