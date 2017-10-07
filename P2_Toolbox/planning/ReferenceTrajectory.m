% Abstract SuperClass for All Trajectory Planning Curves, defining a
% standard set of external access methods and properties so that algorithms
% using one RT (ex TTC) can easily substitute it for another (ex. TCS).
classdef (Abstract) ReferenceTrajectory < handle
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
       
        
        function xx = xs(obj); xx=getXVec(obj); end %Shorthand
        function yy = ys(obj); yy=getYVec(obj); end %Shorthand
        function thth = ths(obj); thth=getThVec(obj); end %Shorthand
        
        function tt = ts(obj); tt=getTVec(obj); end %Shorthand
        function ss = ss(obj); ss=getSVec(obj); end %Shorthand
    end % ReferenceTrajectory <-methods
    
    % Common Access Methods
    methods (Access=public)
        % Plots the trajectory onto the active figure.
        function plot(obj)
            xs = obj.xs();
            ys = obj.ys();
            plot(xs,ys,'r');
            xf = xs(end);% <- there might be some subtlety I overlooked here wrt numSamples in TCS - CWC.
            yf = ys(end);
            r = max([abs(xf) abs(yf)]);
            xlim([-2*r 2*r]);
            ylim([-2*r 2*r]);
        end % #plot
    end % ReferenceTrajectory <-methods
end % Class ReferenceTrajectory