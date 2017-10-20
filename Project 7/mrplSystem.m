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
       
        debugging = struct(...
            'delay_plots', 0, ...   % Whether Transient Velocity Plots for Determining should be Made.
            'error_plots', 0 ...    % Whether Transient Error Plots (from FeedbackController) should be Made.
        );
            delay_plot_data = struct('tv',0, 'rv',0, 't',0);
            delay_error_data = struct('ex',0, 'ey',0, 'eth',0, 'es',0, 't',0);
        
        plot_figure;
        errors;
    end
    
    properties(GetAccess=private, SetAccess=private)
         k_tau = 5.5;%7.0253;%1.4; A.S.S.: 1.95    % Trajectory Time Multiplier for Corrective Time
         % RaspBot-16: Delay:0.164,Ramp:0.05,k_tau:6,vm:0.2
         % RaspBot-17: Delay:0.164,Ramp:0.05,k_tau:5.5,vm:0.2
    end

%% METHODS
    methods
        %% Constructor:
        function obj = mrplSystem(robot_id, startPose)
            %% Setup Internal Data Classes:
            obj.clock = Clock();
            %% Set Initial NullTrajectory: 
            obj.traj_vec = Trajectory_CubicSpiral.planTrajectory(0,0,0,1,2,1);
            obj.traj_vec.is_null = 1;
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
            rt.offsetInitPose();

            tf = Trajectory_Follower(obj.rob, rt);
            tf.fbk_controller.correctiveTime = obj.k_tau;%* rt.getFinalTime();
            
            first_loop = 1;
         	T = 0;
            while (T < tf.rt.getFinalTime()+1)
                if(first_loop)
                    obj.clock = Clock();
                    first_loop = 0;
                end
                
                T = obj.clock.time();
                tf.follow_update_t(T);
                
                obj.update_plotData();
                
                pause(0.01);
            end
            obj.rob.moveAt(0,0);
            obj.rob.core.stop();
            
            %Store completed trajectory
            obj.traj_vec(end+1) = rt;
            %Update plot after completed trajectory
            if(obj.plotting_enabled)
               obj.update_plot();
            end 
        end

        % Helper Function to Update Data used for Debugging Plots
        function update_plotData(obj)
            if obj.debugging.delay_plots
                obj.delay_plot_data(end+1) = struct( ...
                    'tv', rt.V_t(T), ...
                    'rv', obj.rob.measTraj.V_f, ...
                    't', T ...
                );
            end % delay_plots 
            if obj.debugging.error_plots
                ep = tf.fbk_controller.error_poses(end);
                es = norm(ep.poseVec(1:2));
                obj.delay_error_data(end+1) = struct( ...
                    'ex',ep.x, 'ey',ep.Y, 'eth',ep.th, 'es',es, 't',T ...
                );
            end % error_plots?
        end % #update_plotData
        
        % Update All Desired/Active Plots
        function update_plot(obj)
            % DEBUGGING PLOTS:
            if obj.debugging.delay_plots
                tvs = [delay_plot_data(:).tv];
                rvs = [delay_plot_data(:).rv];
                ts = [delay_plot_data(:).t];
                figure();
                    hold on
                        plot(ts,tvs);
                        plot(ts,rvs);
                    hold off
                    title('Transient Velocity Plots for Determining Delay');
                    xlabel('Time [s]');
                    ylabel('Velocity at Time [m/s]');
                    legend('Reference Trajectory', 'Robot Trajectory');
            end % delay_plots?
            if obj.debugging.error_plots
                exs = [delay_error_data(:).ex];
                eys = [delay_error_data(:).ey];
                eths = [delay_error_data(:).eth];
                ess = [delay_error_data(:).es];
                ts = [delay_error_data(:).t];
                figure();
                    hold on
                        plot(ts,exs);
                        plot(ts,eys);
                        plot(ts,eths);
                        plot(ts,ess);
                    hold off
                    title('Transient Error Plots');
                    xlabel('Time [s]');
                    ylabel('Error [m]');
                    legend( ...
                        'Alongtrack, \deltax', ...
                        'Crosstrack, \deltay', ...
                        'Heading, \delta\theta', ...
                        'Position, \deltas' ...
                    );
            end % delay_plots?
            
            % TRAJECTORY PLOTS:
            if isempty(obj.plot_figure)
                obj.plot_figure = figure();
            else
                figure(obj.plot_figure);
            end
            clf;
            
            xlabel('World X-Position Relative to Start [m]');
            ylabel('World Y-Position Relative to Start [m]');
            
            %Compute Error:
            last_traj = obj.traj_vec(end);
            tf = last_traj.p_f;
            rf = obj.rob.measTraj.p_f;
            obj.errors(end+1) = norm(rf.poseVec(1:2) - tf.poseVec(1:2));
            avg_error = mean(obj.errors);
            
            title({ ...
                strcat('Trajectory to: ', num2str(tf.X),',',num2str(tf.Y)), ...
                strcat('Error: ', num2str(obj.errors(end)), ' Avg. Errors: ', num2str(avg_error))
            });
            
                % Produce Plots:
                % Collect output handles in vector so that auxiliary plots
                % such as tangents/headings will be ignored in legend.
                plots = obj.rob.measTraj.plot();
                
                i = 2;
                while i <= length(obj.traj_vec)
                    traj = obj.traj_vec(i);
                    if(~traj.is_null)
                        plots(end+1) = traj.plot(); %for loop? - can we vectorize this?
                    end
                i = i+1;
                end
            
            legend(plots, 'Measured Trajectory', 'Reference Trajectory');
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