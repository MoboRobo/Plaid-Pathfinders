% Abstract SuperClass for All Trajectory Planning Curves, defining a
% standard set of external access methods and properties so that algorithms
% using one RT (ex TTC) can easily substitute it for another (ex. TCS).
classdef (Abstract) ReferenceTrajectory < Trajectory
    properties(Abstract)
        send_delay;     % s, Delay from Command Send to When the Robot Begins Executing it.
        
        N_samples;      % Number of Samples in the Trajectory
    end % ReferenceTrajectory <-properties(Abstract)
    
    methods(Abstract)
        
        % Compute Everything Necessary to Create a Follow-able Path
        compute(obj);
        
        % Velocity at Time:
        V = getVAtTime(obj,t);
        % Angular Velocity at Time:
        om = getOmegaAtTime(obj,t);
        % Path Curvature at Time:
        K = getCurvAtTime(obj,t);
        
        % Pose at Time:
        p = getPoseAtTime(obj,t);
        
        %For Inverting Parameterization if Necessary (Computationally
        %heavy)
        % Path Length at t:
        s = getSAtTime(obj,t);
        % Time at Path Length:
        t = getTAtDist(obj,s);
        
        % Pose at Path Length
        p = getPoseAtDist(obj,s);
        
        % Curvature at Path Length:
        K = getCurvAtDist(obj,s);
        % Angular Velocity at Path Length:
        om = getOmegaAtDist(obj,s);
        % Velocity at Path Length:
        V = getVAtDist(obj,s);
        
        
        %Return the Pose at the End of the Path:
        pf = getFinalPose(obj);
        %Return the Elapsed Time at the End of the Path:
        tf = getFinalTime(obj);
        %Return the Path Length Covered at the End of the Path:
        sf = getFinalDist(obj);
        
        %Returns Vector of All X-Positions:
        xs = getXVec(obj);
        %Returns Vector of All Y-Positions:
        ys = getYVec(obj);
        %Returns Vector of All Headings:
        ths = getThVec(obj);
        %Returns Vector of All Times:
        ts = getTVec(obj);
        %Returns Vector of All Path Lengths:
        ss = getSVec(obj);
        
    end % ReferenceTrajectory <-methods(Abstract)
    
end % Class ReferenceTrajectory