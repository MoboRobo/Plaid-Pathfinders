classdef Trajectory_Null < ReferenceTrajectory
    %Null Trajectory Implements a Null Trajectory.
    
    % Inherited Abstract Methods (from ReferenceTrajectory)
    properties
        send_delay = 0;     % s, Delay from Command Send to When the Robot Begins Executing it.
        
        N_samples;      % Number of Samples in the Trajectory
    end % Trajectory_Null <-properties(Abstract)

    methods(Access = public)
        
        function obj = Trajectory_Null()
            % Check if all Abstract Methods are Implemented:
            meta.abstractDetails(?Trajectory_Null)
        end 
        
        function s  = getDistAtDist(~,~)
            s = 0;
        end
        
        function K  = getCurvAtDist(~,~)
            K = 0;
        end
        
        function p  = getPoseAtDist(~,~)
            p = pose(0,0,0);
        end
        
        function p  = getFinalPose(~)
            p = pose(0,0,0);  
        end
        
        function time  = getTrajectoryDuration(~)
            time = 0;  
        end
        
        function dist  = getTrajectoryDistance(~)
            dist = 0;  
        end
        
        function V  = getVAtTime(~,~)
            V = 0;
        end
            
        function w  = getwAtTime(~,~)
            w = 0;
        end
            
        function p  = getPoseAtTime(~,~)
            p = 0;
        end  
    end
    
    
    % Inherited Abstract Methods (from ReferenceTrajectory)
    methods
        
        % Compute Everything Necessary to Create a Follow-able Path
        function compute(~)
            % Nothing to Compute
        end
        
        function offsetInitPose(~)
            % Nothing to Offset.
        end
        
        % Angular Velocity at Time:
        function om = getOmegaAtTime(~,~)
            om = 0;
        end
        % Path Curvature at Time:
        function K = getCurvAtTime(~,~)
            K = 0;
        end
        
        %For Inverting Parameterization if Necessary (Computationally
        %heavy)
        % Path Length at t:
        function s = getSAtTime(~,~)
            s = 0;
        end
        % Time at Path Length:
        function t = getTAtDist(~,~)
            t = 0;
        end
        
        % Angular Velocity at Path Length:
        function om = getOmegaAtDist(~,~)
            om = 0;
        end
        % Velocity at Path Length:
        function V = getVAtDist(~,~)
            V = 0;
        end
        
        
        %Return the Elapsed Time at the End of the Path:
        function tf = getFinalTime(~)
            tf = 0;
        end
        %Return the Path Length Covered at the End of the Path:
        function sf = getFinalDist(~)
            sf = 0;
        end
        %Returns the Velocity at the End of the Path:
        function vf = getFinalVelocity(obj); vf = obj.V_t(obj.t_f); end
        %Returns the Velocity at the End of the Path:
        function omf = getFinalOmega(obj); omf = obj.om_t(obj.t_f); end
        %Returns the Velocity at the End of the Path:
        function Kf = getFinalCurv(obj); Kf = obj.K_t(obj.t_f); end
        
        %Returns Vector of All X-Positions:
        function xs = getXVec(~)
            xs = [0 0 0];
        end
        %Returns Vector of All Y-Positions:
        function ys = getYVec(~)
            ys = [0 0 0];
        end
        %Returns Vector of All Headings:
        function ths = getThVec(~)
            ths = [0 0 0];
        end
        %Returns Vector of All Times:
        function ts = getTVec(~)
            ts = [0 0 0];
        end
        %Returns Vector of All Path Lengths:
        function ss = getSVec(~)
            ss = [0 0 0];
        end
        %Returns Vector of All Poses:
        function ps = getPVec(~)
            ps = [pose(0,0,0), pose(0,0,0), pose(0,0,0)];
        end
        
    end % ReferenceTrajectory <-methods(Abstract)
end