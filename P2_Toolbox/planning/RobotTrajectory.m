% Trajectory Class for a Transient Robot Trajactory which is Updated over
% Time instead of being calculated at One Singular Point in Time (as with a
% ReferenceTrajectory)
classdef RobotTrajectory < Trajectory
    properties(Constant)
        max_queue_size = 10000; % Max Size of Each Data Set
    end
    properties(GetAccess=public, SetAccess=private)
        data_V;         % Set of All Velocities
        data_om;        % Set of All Angular Velocities
        data_K;         % Set of All Curvatures
        
        data_poses;     % Set of All Poses
        
        data_s;         % Set of All Path Distances/Lengths
        data_t;         % Set of All Time Stamps
    end % RobotTrajectory <- properties(public,private)
    
    % Class-Specific Methods
    methods
        % Initializes a RobotTrajectory, with all the arguments as
        % optional parameters of initial condition.
        function obj = RobotTrajectory(init_pose, init_V,init_om, init_s, init_t)
            % Check if all Abstract Methods are Implemented:
            meta.abstractDetails(?RobotTrajectory)
            
            if nargin>0
                obj.data_poses = slidingFifo(RobotTrajectory.max_queue_size, init_pose);
            else
                obj.data_poses = slidingFifo(RobotTrajectory.max_queue_size, pose(0,0,0));
            end
            
            if nargin>1
                obj.data_V = slidingFifo(RobotTrajectory.max_queue_size, init_V);
            else
                obj.data_V = slidingFifo(RobotTrajectory.max_queue_size, 0);
            end
            
            if nargin>2
                obj.data_om = slidingFifo(RobotTrajectory.max_queue_size, init_om);
                init_K = init_om / init_V;
                obj.data_K = slidingFifo(RobotTrajectory.max_queue_size, init_K);
            else
                obj.data_om = slidingFifo(RobotTrajectory.max_queue_size, 0);
                obj.data_K = slidingFifo(RobotTrajectory.max_queue_size, 0);
            end
                
            if nargin>3
                obj.data_s = slidingFifo(RobotTrajectory.max_queue_size, init_s);
            else
                obj.data_s = slidingFifo(RobotTrajectory.max_queue_size, 0);
            end
                
            if nargin>4
                obj.data_t = slidingFifo(RobotTrajectory.max_queue_size, init_t);
            else
                obj.data_t = slidingFifo(RobotTrajectory.max_queue_size);
            end
        end
    end
    
    methods
        % Interpolation Function with optional specification for the value
        % of the function when the input is under bounds, v_under, and over
        % bounds, v_over.
        function v = getInt(~, xs,vs, x, v_under,v_over)
            v_ub = vs(0);
            v_ob = vs(end);
            if nargin>4
                v_ub = v_under;
            end
            if nargin>5
                v_ob = v_over;
            end
            
            if x < xs(1)
                v = v_ub;
            elseif x > xs(end)
                v = v_ob;
            else
                v = interp1(xs,vs, x, 'pchip','extrap');
            end
        end% #getInt
    end % RobotTrajectory <- methods
    
    % Implementation of Abstract Methods
    methods
        
        % Velocity at Time:
        function V = getVAtTime(obj,t)
            ts = obj.ts();
            Vs = obj.data_V.vec();
            V = obj.getInt(ts,Vs, t);
        end
        % Angular Velocity at Time:
        function om = getOmegaAtTime(obj,t)
            ts = obj.ts();
            oms = obj.data_om.vec();
            om = obj.getInt(ts,oms, t);
        end
        % Path Curvature at Time:
        function K = getCurvAtTime(obj,t)
            ts = obj.ts();
            Ks = obj.data_K.vec();
            K = obj.getInt(ts,Ks, t);
        end
        
        % Pose at Time:
        function p = getPoseAtTime(obj,t)
            ts = obj.ts();
            
            x = obj.getInt(ts,obj.xs(), t);
            y = obj.getInt(ts,obj.ys(), t);
            th = obj.getInt(ts,obj.ths(), t);
            
            p = poses(x,y,th);
        end
        
        %For Inverting Parameterization if Necessary (Computationally
        %heavy)
        % Path Length at t:
        function s = getSAtTime(obj,t)
            ts = obj.ts();
            ss = obj.ss();
            s = obj.getInt(ts,ss, t);
        end
        % Time at Path Length:
        function t = getTAtDist(obj,s)
            ts = obj.ts();
            ss = obj.ss();
            t = obj.getInt(ss,ts, s);
        end
        
        % Pose at Path Length
        function p = getPoseAtDist(obj,s)
            ss = obj.ss();
            
            x = obj.getInt(ss,obj.xs(), s);
            y = obj.getInt(ss,obj.ys(), s);
            th = obj.getInt(ss,obj.ths(), s);
            
            p = poses(x,y,th);
        end
        
        % Curvature at Path Length:
        function K = getCurvAtDist(obj,s)
            ss = obj.ss();
            Ks = obj.data_K.vec();
            K = obj.getInt(ss,Ks, s);
        end
        % Angular Velocity at Path Length:
        function om = getOmegaAtDist(obj,s)
            ss = obj.ss();
            oms = obj.data_om.vec();
            om = obj.getInt(ss,oms, s);
        end
        % Velocity at Path Length:
        function V = getVAtDist(obj,s)
            ss = obj.ss();
            Vs = obj.data_V.vec();
            V = obj.getInt(ss,Vs, s);
        end
        
        
        %Return the Pose at the End of the Path:
        function pf = getFinalPose(obj)
            ps = obj.data_poses.vec();
            pf = ps(end);
        end
        %Return the Elapsed Time at the End of the Path:
        function tf = getFinalTime(obj)
            ts = obj.data_t.vec();
            tf = ts(end);
        end
        %Return the Path Length Covered at the End of the Path:
        function sf = getFinalDist(obj)
            ss = obj.data_s.vec();
            sf = ss(end);
        end
        %Returns the Velocity at the End of the Path:
        function vf = getFinalVelocity(obj)
            vs = obj.data_V.vec();
            vf = vs(end);
        end
        %Returns the Velocity at the End of the Path:
        function omf = getFinalOmega(obj)
            oms = obj.data_om.vec();
            omf = oms(end);
        end
        %Returns the Velocity at the End of the Path:
        function Kf = getFinalCurv(obj)
            Ks = obj.data_K.vec();
            Kf = Ks(end);
        end
        
        %Returns Vector of All X-Positions:
        function xs = getXVec(obj)
            ps = obj.data_poses.vec();
            xs = [ps.poseVec]; xs = xs(1,:);
        end
        %Returns Vector of All Y-Positions:
        function ys = getYVec(obj)
            ps = obj.data_poses.vec();
            ys = [ps.poseVec]; ys = ys(2,:);
        end
        %Returns Vector of All Headings:
        function ths = getThVec(obj)
            ps = obj.data_poses.vec();
            ths = [ps.poseVec]; ths = ths(3,:);
        end
        %Returns Vector of All Times:
        function ts = getTVec(obj); ts = obj.data_t.vec(); end
        %Returns Vector of All Path Lengths:
        function ss = getSVec(obj); ss = obj.data_s.vec(); end
        
    end % RobotTrajectory <-methods(Abstract)
end