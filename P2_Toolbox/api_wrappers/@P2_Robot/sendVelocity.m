% Move the Robot with Left and Right Wheel Speeds v_l and v_r,
% Logs the Commands into the Command History hist_commVel, and
% Computes and Logs the Commanded Pose into hist_commPose.
% (wrapper for the RaspBot setVelocity Command.
function sendVelocity(obj, v_l, v_r)
    if (isnan(v_l) || isnan(v_r))
        % Do Nothing
%         ME = MException('P2_Robot:NaNVelocity', ...
%         'sending a nan!');
%         throw(ME);
    else
        obj.core.sendVelocity(v_l, v_r);
        %Compute Amount of Time Elapsed since Previous Command Was Given.
        t = obj.getTime();

        %Now that Previous Command is Done (a new command has been issued),
        %compute IK and robot pose based on how long it was active for.
        [V, omega] = obj.computeIK(v_l, v_r);

        %Update Commanded Odometry:
        obj.commTraj.update(V,omega, t);

        obj.hist_commWheelVel.add(struct('v_l',v_l, 'v_r',v_r));
    end
end % #setVelocity