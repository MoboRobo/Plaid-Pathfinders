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
                obj.data_t = slidingFifo(RobotTrajectory.max_queue_size, 0);
            end
        end
    end
    
    methods
        % Updates the Trajectory with the Velocity Profile (V,om) at the 
        % Given Time.
        function update(obj, V,om,t)
            dt = t - obj.data_t.last();
            obj.data_t.add(t);
            
            if(dt > 0)
                % Mid-Point Algorithm:
                last_pose = obj.data_poses.last();
                
                xs = obj.data_poses.vec;
                if(obj.p_f.X ~= 0)
                    x = [xs(:).poseVec]
                end
                last_V = obj.data_V.last();
                last_om = obj.data_om.last();
                
                new_th = last_pose.th + last_om*dt/2;
                new_x = last_pose.X + last_V*cos(new_th)*dt;
                new_y = last_pose.Y + last_V*sin(new_th)*dt;
                new_th = new_th + last_om*dt/2;

                obj.data_s.add( obj.data_s.last() + last_V*dt );

                obj.data_poses.add( pose(new_x, new_y, new_th) );
            end
            
            obj.data_V.add(V);
            obj.data_om.add(om);
        end % #update

        % Resets the Trajectory to a Single Entry Dataset of its Initial
        % Values.
        function reset(obj)
            obj.data_poses = slidingFifo(RobotTrajectory.max_queue_size, obj.data_poses.first());
            
            obj.data_V = slidingFifo(RobotTrajectory.max_queue_size, obj.data_V.first());
            obj.data_om = slidingFifo(RobotTrajectory.max_queue_size, obj.data_om.first());
            obj.data_K = slidingFifo(RobotTrajectory.max_queue_size, obj.data_K.first());
                
            obj.data_s = slidingFifo(RobotTrajectory.max_queue_size, obj.data_s.first());
            obj.data_t = slidingFifo(RobotTrajectory.max_queue_size, obj.data_t.first());
        end % #reset
        
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
        function pf = getFinalPose(obj); pf = obj.data_poses.last(); end
        %Return the Elapsed Time at the End of the Path:
        function tf = getFinalTime(obj); tf = obj.data_t.last(); end
        %Return the Path Length Covered at the End of the Path:
        function sf = getFinalDist(obj); sf = obj.data_s.last(); end
        %Returns the Velocity at the End of the Path:
        function vf = getFinalVelocity(obj); vf = obj.data_V.last(); end
        %Returns the Velocity at the End of the Path:
        function omf = getFinalOmega(obj); omf = obj.data_om.last(); end
        %Returns the Velocity at the End of the Path:
        function Kf = getFinalCurv(obj); Kf = obj.data_K.last(); end
        
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