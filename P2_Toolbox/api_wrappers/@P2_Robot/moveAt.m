% Moves the Robot with a Speed, V [m/s] (as measured from robot center) and
% Rotational Velocity, omega [rad/s].
function moveAt(obj, V, omega)
    v_r = V + (obj.WHEEL_TREAD/2)*omega;
    v_l = V - (obj.WHEEL_TREAD/2)*omega;

    if (v_l > .5)
        fprintf('v_l greater than max!');
    end
    if (v_r > .5)
        fprintf('v_r greater than max!');
    end
    if (v_l < -.5)
        fprintf('v_l less than min!');
    end
    if (v_r < -.5)
        fprintf('v_r less than min!');
    end
    
    obj.core.sendVelocity(v_l, v_r);
    
    %Compute Amount of Time Elapsed since Previous Command Was Given.
    t = obj.getTime();
    
    %Update Commanded Odometry:
    obj.commTraj.update(V,omega, t);
    
    obj.hist_commWheelVel(end+1) = struct('v_l',v_l, 'v_r',v_r);
    
    obj.curr_V = V;
    obj.curr_omega = omega;
end % #moveAt