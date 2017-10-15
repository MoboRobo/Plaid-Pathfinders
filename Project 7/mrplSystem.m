classdef mrplSystem < handle
     %% PROPERTIES
    properties(GetAccess=public, SetAccess=private)
        clock;              % Internal Time Keeping
        rob;                % Instance of P2_Robot being Controlled
        
        tcs_scale = 1;      % Scale of Cubic Spiral Trajectory Data to be Used
        traj_samples = 201; % Number of Trajectory Samples to Compute in 
                            % Trajectory Planning
        
        traj_vec; 
        plotting_enabled = 1;
    end
    
    properties(GetAccess=private, SetAccess=private)
         k_tau = 1.4;    % Trajectory Time Multiplier for Corrective Time
    end

%% METHODS
    methods
        %% Constructor:
        function obj = mrplSystem(robot_id, startPose)
            %% Setup Internal Data Classes:
            obj.clock = Clock();
            %% Set Initial NullTrajectory: 
            obj.traj_vec(1) = NullTrajectory();
            obj.traj_vec(1).offsetInitPose(startPose);
            %% Setup Robot
            rasp = raspbot(robot_id, startPose);
            obj.rob = P2_Robot( rasp, @()(obj.clock.time()) );
            if(~strcmp(robot_id,'sim'))
                obj.rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
                RangeImage.INDEX_OFFSET(5);
                obj.rob.core.forksDown(); % Prevent Brown-out
            end
            
        end
        
        function goTo_Rel(obj,rel_pose)
            x = rel_pose(0);
            y = rel_pose(1);
            th = rel_pose(2);
            
            rt = Trajectory_CubicSpiral.planTrajectory( ...
                x, y, th, 1, ...
                obj.traj_samples, obj.tcs_scale ...
            );
            
            rt.offsetInitPose(obj.trajVec(end).getFinalPose())

            tf = Trajectory_Follower(obj.rob, rt);
            
            first_loop = 1;
         	T = 0;
            while(T < tf.rt.getFinalTime()+1)
                if(first_loop)
                    obj.clock = Clock();
                    first_loop = 0;
                end
                T = obj.clock.time();
                tf.follow_update_t(T);
                pause(0.01);
            end
            %Update plot after completed trajectory
            if(obj.plotting_enabled == 1)
               obj.updatePlot();
            end      
            %Store completed trajectory 
            obj.trajVec(end+1) = rt;
            
        end

        function updatePlot(obj)
            figure();
            hold on
                for traj = obj.traj_vec
                    traj.plot() %for loop? - can we vectorize this?
                end
                obj.rob.estTraj.plot()
            hold off
            
            axis equal

        end
        
        function plottingOn(obj)
            obj.plotting_enabled = 1;
        end
        function plottingOff(obj)
            obj.plotting_enabled = 0;
        end
    end
    
    
end