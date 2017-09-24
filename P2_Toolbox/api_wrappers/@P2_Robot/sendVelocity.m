% Move the Robot with Left and Right Wheel Speeds v_l and v_r,
% Logs the Commands into the Command History hist_commVel, and
% Computes and Logs the Commanded Pose into hist_commPose.
% (wrapper for the RaspBot setVelocity Command.
function sendVelocity(obj, v_l, v_r)
    obj.core.sendVelocity(v_l, v_r);
    
    %Compute Amount of Time Elapsed since Previous Command Was Given.
    t = toc(obj.on_time);
    dt = t - obj.hist_commTime(end);
    
    %Now that Previous Command is Done (a new command has been issued),
    %compute IK and robot pose based on how long it was active for.
    [V, omega] = obj.computeIK(obj.hist_commVel(end).v_l, obj.hist_commVel(end).v_r);
    
    %Update Commanded Odometry (Mid-Point Algorithm):
    th = obj.hist_commPose(end).th + omega*dt/2;
    x = obj.hist_commPose(end).X + V*cos(th)*dt;
    y = obj.hist_commPose(end).Y + V*sin(th)*dt;
    th = th + omega*dt/2;
    
    obj.hist_commTime(end+1) = t;
    obj.hist_commVel(end+1) = struct('v_l',v_l, 'v_r',v_r, 'V',V, 'om',omega);
    obj.hist_commPose(end+1) = struct('X',x, 'Y',y, 'th',th);
end % #setVelocity