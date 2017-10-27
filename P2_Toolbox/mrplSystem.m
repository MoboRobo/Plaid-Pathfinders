classdef mrplSystem < handle
     %% PROPERTIES
    properties(GetAccess=public, SetAccess=private)
        clock;              % Internal Time Keeping
        rob;                % Instance of P2_Robot being Controlled
        feedback_controller;    % Persistent Feedback Controller Class
        
        tcs_scale = 1;      % Scale of Cubic Spiral Trajectory Data to be Used
        traj_samples = 551; % Number of Trajectory Samples to Compute in 
                            % Trajectory Planning
        
        traj_vec = MakeNullInstance_TCS();
        plotting_enabled = 1;
       
            delay_plot_data = slidingFifo(10000, struct('tv',0, 'rv',0, 't',0));
            delay_error_data = slidingFifo(10000, struct('ex',0, 'ey',0, 'eth',0, 'es',0, 't',0));
        
        plot_figure;
        errors;
    end
    
    properties(GetAccess = public, SetAccess = public)
     debugging = struct(...
            'delay_plots', 0, ...   % Whether Transient Velocity Plots for Determining should be Made.
            'error_plots', 0, ...    % Whether Transient Error Plots (from FeedbackController) should be Made.
            'comm_plots', 0 ...     % Whether Transient Comm plots should be made
        );
    end
    properties(GetAccess=private, SetAccess=private)
         k_tau = 5.5;%7.0253;%1.4; A.S.S.: 1.95    % Trajectory Time Multiplier for Corrective Time
         % RaspBot-16: Delay:0.164,Ramp:0.05,k_tau:6,vm:0.2
         % RaspBot-17: Delay:0.164,Ramp:0.05,k_tau:5.5,vm:0.2
    end

%% METHODS
    methods
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - SETUP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %% Constructor:
        function obj = mrplSystem(robot_id, startPose)
            %% Setup Internal Data Classes:
            obj.clock = Clock();
            %% Set Initial NullTrajectory: 
            obj.traj_vec = MakeNullInstance_TCS();
            %% Setup Robot
            rasp = raspbot(robot_id, [startPose.X; startPose.Y; startPose.th+pi/2.0]);
            obj.rob = P2_Robot( rasp, @()(obj.clock.time()) );
            if(~strcmp(robot_id,'sim'))
                obj.rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
                RangeImage.INDEX_OFFSET(5);
                obj.rob.core.forksDown(); % Prevent Brown-out
            end
            
            obj.feedback_controller = FeedbackController.empty;
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - MOTION CONTROL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        

        %% Get Nearest Line Object Pose
        % Returns the Pose of the Nearest Valid Line Object in the Current
        % Lidar RangeImage in the Robot Frame
        function p_nlo_r = getNearestLineObject(obj)
            if(~obj.rob.laser_state) % If lasers are off
                obj.rob.laserOn(); % Turn Lasers on
                pause(1); % Wait for Lasers to generate RangeImages.
            end
            
            
            r_img = obj.rob.hist_laser.last(); % Get Latest Image.
            while(length(r_img.line_candidates.lengths) == 1 ...
               && r_img.line_candidates.lengths(1) == 0)
            % Loop until valid line candidate is found
                r_img = obj.rob.hist_laser.last(); % Get Latest Image.
                r_img.findLineCandidates(); % (does what it says on the tin)
                pause(0.01); % Relief for Callback to Update
            end
            
            p_los = r_img.line_candidates.poses; % Poses of all Valid Line Objects
            % Get Nearest Line Object Pose to Robot in Robot Frame:
            
            i = 1;
            n = length(p_los);
            p_nlo_r = pose(0,0,0); % needs default value
            min_s = Inf; % Current Minimum Pose Distance
            while i<=n
            % Loop through all p_los to id which has least distance in
            % robot frame.
                p = p_los(i); % Pose being tested
                s = norm([p.X p.Y]);
                if s < min_s && s~=0
                    p_nlo_r = p;
                    min_s = s;
                end
            i = i+1;
            end
        end
        
        function turn_stationary(obj, th)
            Stationary_Turn(obj.rob, th);
        end
        
        
        function goTo_X_Small(obj, x_rel)
            %Anonymous function handle for velocity profile:
            vref = @(~,t)u_ref_trap(t,obj.rob.MAX_ACCEL,0.5*obj.rob.MAX_SPEED,x_rel,0,0);
            omref = @(~,t)0;
            t_f = u_ref_trap(0,obj.rob.MAX_ACCEL,0.5*obj.rob.MAX_SPEED,x_rel,0,1);
            
            ttc = Trajectory_TimeCurve(vref,omref, 0, t_f, obj.traj_samples);
            ttc.init_pose = obj.traj_vec(end).getFinalPose();
            ttc.offsetInitPose();
            
            tf = Trajectory_Follower(obj.rob, ttc);
               
            obj.time_loop(tf, 1);
            
            % Until Mix-Ins are implemented for Trajectory Class Hierarchy,
            % Set traj_vec terminal TTC to equivalent TCS (to preserve
            % homogeneity).
            eq_tcs = Trajectory_CubicSpiral.planTrajectory( ...
                x_rel, 0, 0, 1, ...
                obj.traj_samples, obj.tcs_scale ...
            );
            eq_tcs.init_pose = obj.traj_vec(end).getFinalPose();
            eq_tcs.offsetInitPose();
            obj.traj_vec(end+1) = eq_tcs;
            
            %Update plot after completed trajectory
            if(obj.plotting_enabled)
               obj.update_plot(tf);
            end
            
        end % #goTo_X_Small
        function goTo_th_Small(obj, th_rel)
            %Anonymous function handle for velocity profile:
            omref = @(~,t)u_ref_trap(t,3*obj.rob.MAX_ALPHA/4,0.5*obj.rob.MAX_OMEGA,th_rel,0,0);
            vref = @(~,t)0;
            t_f = u_ref_trap(0,obj.rob.MAX_ALPHA,0.5*obj.rob.MAX_OMEGA,th_rel,0,1);
            
            ttc = Trajectory_TimeCurve(vref,omref, 0, t_f, obj.traj_samples);
            ttc.init_pose = obj.traj_vec(end).getFinalPose();
            ttc.offsetInitPose();
            
            tf = Trajectory_Follower(obj.rob, ttc);
            tf.fbk_controller.isPureTurn = 1;
               
            obj.time_loop(tf, 1);
            
            % Until Mix-Ins are implemented for Trajectory Class Hierarchy,
            % Set traj_vec terminal TTC to equivalent TCS (to preserve
            % homogeneity).
            eq_tcs = Trajectory_CubicSpiral([0 0 0], 3);
            pf = obj.traj_vec(end).getFinalPose();
            eq_tcs.init_pose = pose(pf.X, pf.Y, pf.th+th_rel);
            eq_tcs.offsetInitPose();
            obj.traj_vec(end+1) = eq_tcs;
            
            %Update plot after completed trajectory
            if(obj.plotting_enabled)
               obj.update_plot(tf);
            end
            
        end % #goTo_th_Small
        
        %% Go To Relative Position
        function goTo_Rel(obj,rel_pose)
            x = rel_pose.x;
            y = rel_pose.y;
            th = rel_pose.th;
            
            rt = Trajectory_CubicSpiral.planTrajectory( ...
                x, y, th, 1, ...
                obj.traj_samples, obj.tcs_scale ...
            );
            
            rt.init_pose = obj.traj_vec(end).getFinalPose();
            % if you don't call offsetInitPose, Trajectory automatically
                %transforms each reference pose before handing it to mrpl
            rt.offsetInitPose();
            
            if isempty(obj.feedback_controller)
                tf = Trajectory_Follower(obj.rob, rt);
                obj.feedback_controller = tf.fbk_controller;
            else
                tf = Trajectory_Follower(obj.rob, rt, obj.feedback_controller);
            end
            tf.fbk_controller.correctiveTime = obj.k_tau;%* rt.getFinalTime();
            
            obj.time_loop(tf, 1);
             
            %Store completed trajectory
            obj.traj_vec(end+1) = tf.rt;
            %Update plot after completed trajectory
            if(obj.plotting_enabled)
               obj.update_plot(tf);
            end
            
        end % #goTo_Rel

        % Helper Function Executing a loop commanding the robot to follow
        % the given trajectory follower until the t_buffer seconds after 
        % the tf's reference trajectory's final time.
        function time_loop(obj, tf, t_buffer)
            first_loop = 1;
         	T = 0;
            done = 0;
            while (~done)
                if(first_loop)
                    obj.clock = Clock();
                    first_loop = 0;
                end
                
                T = obj.clock.time();
                tf.follow_update_t(T);
                obj.clock.pause();
                obj.clock.resume();
                
                if(obj.plotting_enabled)
                    obj.update_plotData(tf, T);
                end
                
                if(T > tf.rt.getFinalTime()+t_buffer)
                    done = 1;
                    obj.rob.moveAt(0,0); % Stop Immediately
                end
                
                pause(0.01);
            end
            obj.rob.moveAt(0,0);
            obj.rob.core.stop();
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - PLOTTING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        %% Update Plot Data:
        % Helper Function to Update Data used for Debugging Plots (during
        % loop).
        function update_plotData(obj, tf, T)
            if obj.debugging.delay_plots
                obj.delay_plot_data.add(struct( ...
                    'tv', tf.rt.V_t(T), ...
                    'rv', obj.rob.measTraj.V_f, ...
                    't', T ...
                ));
            end % delay_plots
            if obj.debugging.error_plots
                ep = tf.fbk_controller.error_poses.last();
                es = norm(ep.poseVec(1:2));
                obj.delay_error_data.add(struct( ...
                    'ex',ep.x, 'ey',ep.Y, 'eth',ep.th, 'es',es, 't',T ...
                ));
            end % error_plots?
        end % #update_plotData
        
        %% Update Plot:
        % Update All Desired/Active Plots
        function update_plot(obj, tf)
            % DEBUGGING PLOTS:
            if obj.debugging.delay_plots
                delay_plot_vecdata = obj.delay_plot_data.vec();
                tvs = [delay_plot_vecdata(:).tv];
                rvs = [delay_plot_vecdata(:).rv];
                ts = [delay_plot_vecdata(:).t];
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
                delay_error_vecdata = obj.delay_error_data.vec;
                exs = [delay_error_vecdata(:).ex];
                eys = [delay_error_vecdata(:).ey];
                eths = [delay_error_vecdata(:).eth];
                ess = [delay_error_vecdata(:).es];
                ts = [delay_error_vecdata(:).t];
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
            end % error_plots?
            
            if obj.debugging.comm_plots
                comm_Vs = tf.fbk_controller.comm_V_t.vec();
                comm_Ws = tf.fbk_controller.comm_W_t.vec();
                vs = [comm_Vs(:).comm_v];
                ws = [comm_Ws(:).comm_w];
                ts = [comm_Vs(:).t];
                figure();
                    hold on
                        plot(ts,vs);
                        plot(ts,ws);
                    hold off
                    title('Transient Command Plots');
                    xlabel('Time [s]');
                    ylabel('Command [(m/s)/(rad/s)]');
                    legend( ...
                        'Linear Velocity, v', ...
                        'RotationalVelocity, \deltao' ...
                    );
            end % comm_plots?
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
        
        %% Plotting On/Off
        function plottingOn(obj)
            obj.plotting_enabled = 1;
        end
        function plottingOff(obj)
            obj.plotting_enabled = 0;
        end
        
    end % mrplSystem <- methods
    
    methods(Static)
        function acqPose = acquisitionPose(objPose,...
            robFrontOffset, objFaceOffset, moreOffset, lateralFudge)
            totalDist = robFrontOffset + objFaceOffset + moreOffset;
            x1 = objPose.x;
            y1 = objPose.y;
            th1 = objPose.th;
            x = x1 - totalDist * cos(th1); y = y1 - totalDist*sin(th1)+lateralFudge;
%             
%         p__r_s = pose(0, 0, 0);
%         T__r_s = p__r_s.bToA();
%         p__s_o = objPose;
%         T__s_o = p__s_o.aToB();
% 
%         totalDist = robFrontOffset + objFaceOffset + moreOffset;
%         Dx_ = - totalDist; Dy_ = 0;
%         D_th_ = 0;
%         t__o_g = pose(Dx_, Dy_, D_th_);
%         T__o_g = t__o_g.aToB();
% 
%         finalTransformation = T__r_s * T__s_o * T__o_g;
% 
%         result = (finalTransformation * [objPose.X; objPose.Y; 1])';
%         x = result(1); y = result(2);
        acqPose = pose(x, y, objPose.th);
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - DATA MANIPULATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        

    end % mrplSystem <- methods(static)
    
end