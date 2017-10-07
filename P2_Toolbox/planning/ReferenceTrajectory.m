% Abstract SuperClass for All Trajectory Planning Curves, defining a
% standard set of external access methods and properties so that algorithms
% using one RT (ex TTC) can easily substitute it for another (ex. TCS).
classdef (Abstract) ReferenceTrajectory
    properties(Abstract)
        send_delay;     % s, Delay from Command Send to When the Robot Begins Executing it.
        
        N_samples;      % Number of Samples in the Trajectory
        resolution;     % s, Separation between Times
        
        t_init;         % s, Curve Seed Time
        t_fin;          % s, Curve End Time
    end % ReferenceTrajectory <-properties(Abstract)
    
    methods (Abstract)
        
        compute(obj);
        
        V = getVAtTime(obj,t);%         - Velocity at Time
        om = getOmegaAtTime(obj,t);%    - Angular Velocity at Time
        K = getCurvAtTime(obj,t);%      - Path Curvature at Time
        
        p = getPoseAtTime(obj,t);%      - Pose at Time
        
        %For Inverting Parameterization if Necessary (Computationally
        %heavy)
        s = getSAtTime(obj,t);%         - Path Length at t
        t = getTAtDist(obj,t);%         - Time at Path Length
        
        p = getPoseAtDist(obj,s);%      - Pose at Path Length
        
        K = getCurvAtDist(obj,s);%      - Curvature at Path Length
        om = getOmegaAtDist(obj,s);%    - Angular Velocity at Path Length
        V = getVAtDist(obj,s);%         - Velocity at Path Length
        
        
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
    
    % Short-hand methods:
    methods (Access=public)
        function V = V_t(obj, t); V = obj.getVAtTime(t); end
        function om = om_t(obj, t); om = obj.getOmegaAtTime(t); end
        function K = K_t(obj, t); K = obj.getCurvAtTime(t); end
        
        function p = p_t(obj, t); p = obj.getPoseAtTime(t); end
        
        function s = s_t(obj, t); s = obj.getSAtTime(t); end
        function t = t_s(obj, s); t = obj.getTAtDist(s); end
        
        function p = p_s(obj, s); p = obj.getPoseAtDist(s); end
        
        function K = K_s(obj, s); K = obj.getCurvAtDist(s); end
        function om = om_s(obj, s); om = obj.getOmegaAtDist(s); end
        function V = V_s(obj, s); V = obj.getVAtDist(s); end
    end % ReferenceTrajectory <-methods
    
    % Common Access Methods
    methods (Access=public)
        % Plots the trajectory onto the active figure.
        function plot(obj)
            xs = obj.getXVec();
            ys = obj.getYVec();
            plot(xs,ts,'r');
            xf = xs(end);% <- there might be some subtlety I overlooked here wrt numSamples in TCS - CWC.
            yf = ys(end);
            r = max([abs(xf) abs(yf)]);
            xlim([-2*r 2*r]);
            ylim([-2*r 2*r]);
        end % #plot
    end % ReferenceTrajectory <-methods
end % Class ReferenceTrajectory