% Calculates a Constant Curvature Trajectory to the Point at
% (x,y) relative to the robot, with bearing th, and then moves along it at 
% velocity, V. [See the note in the P2_Robot Class for coordinate 
% standards].
% It's recommended that |th|>pi/2 (only angles from in front of the robot).
% l - [meters] Distance to target.
% th - [rad] Bearing of target in radians.
function trajectory_goTo(obj, V, l, th)
    %l = sqrt(x^2 + y^2); % Distance to Point
    
    l_go = l;
    V_go = V;
    th_targ = th;
    
    if(l_go < 0)
    % Turn a negative distance problem into one of hitting a target behind
    % the robot (while driving backwards)
        th_targ = th_targ-pi;
        l_go = -l_go;
    end
    
    if(abs(th_targ) > pi/2) % if target is behind the robot
        V_go = -V_go; % Drive backwards
        l_go = -l_go; % ...towards a target behind the vehicle
        % ...that shares the bearing opposite of what was given
        th_targ = th_targ - pi;
        th_targ = mod(th_targ,pi);
    end
    
    k = 2 * sin(th_targ/2) / l_go; % radius of curvature = l/crd(th). K = 1/R
    omega = k*V_go;
%     
%     if(l < 0) % if distance to object is -ve, back up.
%     % turn a negative distance problem into a negative velocity problem
%         l_go = -l_go;
%         V_go = -V_go;
%     end % l < 0
%     
%     if(V_go < 0)
%     % turn a negative velocity problem into a problem of navigating to a
%     % point behind the robot
%         V_go = -V_go;
%         th_targ = th - pi;
%     end % V_go < 0
%     
%     k = 2 * sin(th_targ/2) / l_go; % radius of curvature = l/crd(-th). K = 1/R
%     
%     if(abs(th) > pi/2) % Target is behind robot
%         V_go = -V_go;
%         th_targ = th - pi;
%         k = -2 * sin(th_targ/2) / l_go; % radius of curvature = l/crd(-th). K = 1/R
%     end% |th| > pi/2?
%     
%     omega = -V_go*k;

    obj.moveAt(V_go, omega);
end % #trajectory_goTo