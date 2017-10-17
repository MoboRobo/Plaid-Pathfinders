classdef mrplSystem < handle
     %% PROPERTIES
    properties(GetAccess=public, SetAccess=private)
        clock;              % Internal Time Keeping
        rob;                % Instance of P2_Robot being Controlled
        
        tcs_scale = 1;      % Scale of Cubic Spiral Trajectory Data to be Used
        traj_samples = 201; % Number of Trajectory Samples to Compute in 
                            % Trajectory Planning
        
        traj_vec = Trajectory_CubicSpiral.empty; 
        plotting_enabled = 1;
        
        plot_figure;
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
            obj.traj_vec = Trajectory_CubicSpiral.planTrajectory(0,0,0,1,2,1);
            obj.traj_vec.init_pose = startPose;
            obj.traj_vec.offsetInitPose();
            %% Setup Robot
            rasp = raspbot(robot_id, [startPose.X; startPose.Y; startPose.th+pi/2.0]);
            obj.rob = P2_Robot( rasp, @()(obj.clock.time()) );
            if(~strcmp(robot_id,'sim'))
                obj.rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
                RangeImage.INDEX_OFFSET(5);
                obj.rob.core.forksDown(); % Prevent Brown-out
            end
            
        end
        
        function goTo_Rel(obj,rel_pose)
            x = rel_pose.x;
            y = rel_pose.y;
            th = rel_pose.th;
            
            rt = Trajectory_CubicSpiral.planTrajectory( ...
                x, y, th, 1, ...
                obj.traj_samples, obj.tcs_scale ...
            );
            
            rt.init_pose = obj.traj_vec(end).getFinalPose();
            rt.offsetInitPose()

            tf = Trajectory_Follower(obj.rob, rt);
            tf.fbk_controller.correctiveTime = obj.k_tau * rt.getFinalTime();
            
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
            if(obj.plotting_enabled)
               obj.updatePlot();
            end      
            %Store completed trajectory 
            obj.traj_vec(end+1) = rt;
        end

        function updatePlot(obj)
            if isempty(obj.plot_figure)
                obj.plot_figure = figure();
                pf = obj.rob.measTraj.p_f;
                title(strcat('Trajectory to: ', num2str(pf.X),',',num2str(pf.Y)));
                xlabel('World X-Position Relative to Start [m]');
                ylabel('World Y-Position Relative to Start [m]');
            end
            figure(obj.plot_figure);
                obj.rob.measTraj.plot();
                
                i = 1;
                while i < length(obj.traj_vec)
                    traj = obj.traj_vec(i);
                    traj.plot(); %for loop? - can we vectorize this?
                i = i+1;
                end
            
            legend('Measured Trajectory', 'Reference Trajectory');
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