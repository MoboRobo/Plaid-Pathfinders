% Moves the Robot with a Speed, V [m/s] (as measured from robot center) and
% Rotational Velocity, omega [rad/s].
function moveAt(obj, V, omega)
    v_l = V - (obj.WHEEL_TREAD/2)*omega;
    v_r = V + (obj.WHEEL_TREAD/2)*omega;
    
    %If the robot is simulated, the sendVelocity command takes 
    % (v_left,v_right); if real, it takes (v_right,v_left).
    obj.core.sendVelocity(v_l,v_r);
    % sim?
    
    obj.curr_V = V;
    obj.curr_omega = omega;
end % #moveAt