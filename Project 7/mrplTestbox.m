% Basic Container Shell Version of mrplSystem for use of testing robot
% class.
classdef mrplTestbox
    %% PROPERTIES
    properties(GetAccess=public, SetAccess=private)
        clock;              % Internal Time Keeping
        rob;                % Instance of P2_Robot being Controlled
        
        tcs_scale = 1;      % Scale of Cubic Spiral Trajectory Data to be Used
        traj_samples = 201; % Number of Trajectory Samples to Compute in 
                            % Trajectory Planning
    end % mrplTestbox <- properties(public,private)
    
    properties(GetAccess=private, SetAccess=private)
        k_tau = 1.4;    % Trajectory Time Multiplier for Corrective Time
    end
    
    %% METHODS
    methods
        %% Constructor:
        function obj = mrplTestbox(robot_id)
            close all;
            rosshutdown
            rosinit
            
            %% Setup Internal Data Classes:
            obj.clock = Clock();
            
            %% Setup Robot
            rasp = raspbot(robot_id, [0; 0; pi/2])
            obj.rob = P2_Robot( rasp, @()(obj.clock.time()) );
            if(~strcmp(robot_id,'sim'))
                obj.rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
                RangeImage.INDEX_OFFSET(5);
                obj.rob.core.forksDown(); % Prevent Brown-out
            end
            
        end
        
        %% Create Trajectory Follower:
        % Instantiates and Sets up a Trajectory Follower to Drive the Robot
        % along the Given Reference Trajectory.
        function tf = createTrajectoryFollower(obj,rt)
            tf = Trajectory_Follower(obj.rob, rt);
                %tf.fbk_trim = 1;
                tf.fbk_controller.correctiveTime = obj.k_tau * rt.getFinalTime();
        end % #createTrajectoryFollower
        
        %% Test Case:
        % Tests all Basic Functionality of mrpl and its members.
        function test(obj)
            rt = Trajectory_CubicSpiral.planTrajectory( ...
                0.3048,0.3048,0.0, 1, ...
                obj.traj_samples, obj.tcs_scale ...
            );
            tf = obj.createTrajectoryFollower(rt);
            
            first_loop = 1;
         	T = 0;
            while (T < tf.rt.getFinalTime()+1 )%&& (~within(E_head,0.01,0) || T < 0.9*tf.rt.getFinalTime()))
                if(first_loop)
                    obj.clock = Clock();
                first_loop = 0;
                end

                T = obj.clock.time();
                tf.follow_update_t(T);
                
                pause(0.01); % CPU Relief
            end
            
            pf = rt.getFinalPose();
            figure();
            title(strcat('Trajectory to: ', num2str(pf.X),',',num2str(pf.Y)));
            xlabel('World X-Position Relative to Start [m]');
            ylabel('World Y-Position Relative to Start [m]');
            hold on
                obj.rob.measTraj.plot();
                rt.plot();
            hold off
            legend('Measured Trajectory', 'Reference Trajectory');
            axis equal
        end % #test
    end % mrplTestbox <- methods
end