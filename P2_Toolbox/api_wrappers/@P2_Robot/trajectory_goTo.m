% Calculates a Constant Curvature Trajectory to the Point at
% (x,y) relative to the robot, with bearing th, and then moves along it at 
% velocity, V. [See the note in the P2_Robot Class for coordinate 
% standards].
% x,y in meters, th in radians.
function trajectory_goTo(obj, V, x, y, th)
    l = sqrt(x^2 + y^2); % Distance to Point
    
    % Calculate Curvature of Path Connecting Two Points:
    k = 2 * sin(th/2) / l; % radius of curvature = l/crd(th). K = 1/R
    
    omega = V*k;
    obj.moveAt(V, omega);
end % #trajectory_goTo