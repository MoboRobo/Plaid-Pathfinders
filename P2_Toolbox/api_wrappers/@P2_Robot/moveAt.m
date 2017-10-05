% Moves the Robot with a Speed, V [m/s] (as measured from robot center) and
% Rotational Velocity, omega [rad/s].
function moveAt(obj, V, omega)
    v_r = V + (obj.WHEEL_TREAD/2)*omega;
    v_l = V - (obj.WHEEL_TREAD/2)*omega;

    if (v_l > .5 || v_r > .5) 
        fprintf('AAAAAAAA');
    end
    obj.sendVelocity(v_l,v_r);
    
    obj.curr_V = V;
    obj.curr_omega = omega;
end % #moveAt