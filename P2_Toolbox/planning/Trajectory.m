% Abstract SuperClass for All Trajectory Paths (Signals and Curves), both 
% pre-planned and Transient, defining a standard set of external access 
% methods and properties so that algorithms using one RT (ex TTC) can 
% easily substitute it for another (ex. TCS).
classdef (Abstract) Trajectory < handle
    properties(GetAccess=public, SetAccess=public)
    end % Trajectory <- properties(public,public)
    
    % Pseudo-Static Parameters:
    methods(Sealed, Static)
        % Whether to Plot Thetas in Plot of Trajectory. Sets to v if
        % given an argument.
        function ret = plot_thetas(v)
            persistent val;
            if isempty(val)
                val = 0; % default value
            end % isempty?
            if nargin > 0
                val = v; % Set Value when given Argument
            end % nargin>0?
            ret = val;
        end % #plot_thetas
        
        % Transforms a Pose from Path-Relative Coordinates to World 
        % Coordinates given an initial position, p0
        function pw = poseToWorld(pr, p0)
            transformationMatrix = p0.bToA();
            result = (transformationMatrix * [pr.X; pr.Y; 1])';
            x = result(1); y = result(2);
            offsetTh = p0.th;
            pw = pose(x, y, ...
                atan2(sin(pr.th + offsetTh), cos(pr.th + offsetTh)));
        end % #poseToWorld
    end % Trajectory <- methods(sealed,static)
    
    methods(Abstract)
        
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
        %Returns the Velocity at the End of the Path:
        vf = getFinalVelocity(obj);
        %Returns the Velocity at the End of the Path:
        omf = getFinalOmega(obj);
        %Returns the Velocity at the End of the Path:
        Kf = getFinalCurv(obj);
        
        %Returns Vector of All X-Positions:
        xs = getXVec(obj);
        %Returns Vector of All Y-Positions:
        ys = getYVec(obj);
        %Returns Vector of All Headings:
        ths = getThVec(obj);
        
        %Returns Vector of All Velocities:
        Vs = getVVec(obj);
        
        %Returns Vector of All Times:
        ts = getTVec(obj);
        %Returns Vector of All Path Lengths:
        ss = getSVec(obj);
        
    end % ReferenceTrajectory <-methods(Abstract)
    
    % Short-hand methods:
    methods (Sealed, Access=public)
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
        
        function vv = Vs(obj); vv=getVVec(obj); end
        
        function tt = ts(obj); tt=getTVec(obj); end %Shorthand
        function ss = ss(obj); ss=getSVec(obj); end %Shorthand
        
        %Return the Pose at the End of the Path:
        function f = p_f(obj); f = obj.getFinalPose(); end
        %Return the Elapsed Time at the End of the Path:
        function f = t_f(obj); f = obj.getFinalTime(); end
        %Return the Path Length Covered at the End of the Path:
        function f = s_f(obj); f = obj.getFinalDist(); end
        
        %Returns the Velocity at the End of the Path:
        function f = V_f(obj); f = obj.getFinalVelocity(); end
        %Returns the Velocity at the End of the Path:
        function f = om_f(obj); f = obj.getFinalOmega(); end
        %Returns the Velocity at the End of the Path:
        function f = K_f(obj); f = obj.getFinalCurv(); end
    end % Trajectory <-methods
    
    % Common Access Methods
    methods (Access=public)
        % Plots the trajectory onto the active figure, returns handle.
        function pl = plot(obj)
        hold on
            xs = obj.xs();
            ys = obj.ys();
            ths = obj.ths();
            ss = obj.ss();
            
            pl = plot(-ys,xs);
            
            if Trajectory.plot_thetas
                num_thetas = 30; % Number of Thetas to Plot
                
                arc_spacing = obj.s_f / num_thetas; % Arc Distance between each Theta
                last_theta_s = -arc_spacing; % Arc Dist. at Plotting of Last Theta
                
%                 xs = xs(ss>0);
%                 ys = ys(ss>0);
%                 ths = ths(ss>0);
%                 ss = ss(ss>0);
%                 
%                 n = 1;
%                 while n<=num_thetas
%                     s = n*arc_spacing;
%                     
%                     try
%                         x0 = interp1(ss,xs,s, 'pchip');
%                         y0 = interp1(ss,ys,s, 'pchip');
%                         th0 = interp1(ss,ths,s, 'pchip');
% 
%                         x1 = x0 + 1.5*arc_spacing*cos(th0);
%                         y1 = y0 + 1.5*arc_spacing*sin(th0);
%                         plot(-[y0 y1], [x0 x1], 'g');
%                     catch error
%                         % Gridded Interpolant Error
%                     end
%                 n = n+1;
%                 end
                
                i = 1;
                while i<length(xs)
                    if ss(i) - last_theta_s > arc_spacing
                        x0 = xs(i);
                        y0 = ys(i);
                        x1 = x0 + 1.5*arc_spacing*cos(ths(i));
                        y1 = y0 + 1.5*arc_spacing*sin(ths(i));
                        plot(-[y0 y1], [x0 x1], 'g');
                        last_theta_s = ss(i);
                    end
                i = i+1;
                end
            end
            axis equal;
        hold off
%             pf = obj.getFinalPose();
%             xf = pf.X;
%             yf = pf.Y;
%             
%             r = max([abs(xf) abs(yf)]);
%             if(r ~= 0)
%                 k_r = 1.2;
%                 xlim([-k_r*r k_r*r]);
%                 ylim([-k_r*r k_r*r]);
%             end
        end % #plot
    end % Trajectory <-methods
end % Class Trajectory