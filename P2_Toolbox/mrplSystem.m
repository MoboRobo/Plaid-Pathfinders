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
        start_poses = []; % << I N   W O R L D   C O O R D I N A T E S >>
        plotting_enabled = 1;
       
            delay_plot_data = slidingFifo(10000, struct('tv',0, 'rv',0, 't',0));
            delay_error_data = slidingFifo(10000, struct('ex',0, 'ey',0, 'eth',0, 'es',0, 't',0));
        
        plot_figure;
        errors;
    end

    properties(GetAccess = public, SetAccess = public)
        interface = RobotInterface.empty; % Interface Controller for this System
        
        debugging = struct(...
            'delay_plots', 1, ...   % Whether Transient Velocity Plots for Determining should be Made.
            'error_plots', 0, ...    % Whether Transient Error Plots (from FeedbackController) should be Made.
            'comm_plots', 0 ...     % Whether Transient Comm plots should be made
        );

        laser_plotting_data = struct( ...
            'delay_width',0.1, ...  % Min amount of time /between/ lidar plots
            'last_time',0, ...      % Last time lidar data was plotted.
... %           'new_data',0, ...       % Flag for Availability of New Lidar Data 
... %             'raw_data',0, ...       % Last Captured Raw Lidar Data
            'colorize',0, ...       % Whether Plotted Data should be Colorized (by range)
            'curr_fig',0, ...       % Current Plotting Figure
            'lidar_plot',0 ...      % Current Lidar Plot Handle
        );
    end
    properties(GetAccess=private, SetAccess=private)
         k_tau = 2;%7.0253;%1.4; A.S.S.: 1.95    % Trajectory Time Multiplier for Corrective Time
         %% 
         %% 
         % ^-6.
         % RaspBot-16: Delay:0.164,Ramp:0.05,k_tau:6,vm:0.2
         % RaspBot-17: Delay:0.164,Ramp:0.05,k_tau:5.5,vm:0.2
    end

%% METHODS
    methods
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - SETUP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %% Constructor:
        function obj = mrplSystem(robot_id, startPose, world_map)
            %% Setup Internal Data Classes:
            obj.clock = Clock();
            %% Set Initial NullTrajectory: 
            obj.traj_vec = MakeNullInstance_TCS();
            %% Setup Robot
            rasp = raspbot(robot_id, [startPose.X; startPose.Y; startPose.th+pi/2.0]);
            
            if nargin>2
                wm = world_map;
            else
                wm = WorldMap([0 0]); %% Default empty bounds
            end
            
            obj.rob = P2_Robot( rasp, startPose, wm, @()(obj.clock.time()) );
            if(~strcmp(robot_id,'sim'))
                obj.rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
                RangeImage.INDEX_OFFSET(5);
                obj.rob.core.forksDown(); % Prevent Brown-out
            end
            
            if nargin>2
                if(~obj.rob.laser_state) % If lasers are off
                    obj.rob.laserOn(); % Turn Lasers on
                    pause(1); % Wait for Lasers to generate RangeImages.
                end
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
            
            % Plot RangeImages until Object is Acquired:
            obj.init_lidar_staticPlotting();
            
            r_img = obj.rob.hist_laser.last(); % Get Latest Image.
            
            % Loop until valid line candidate is found
            while(length(r_img.line_candidates.lengths) == 1 ...
               && r_img.line_candidates.lengths(1) == 0)
                r_img = obj.rob.hist_laser.last(); % Get Latest Image.
                
                obj.update_lidar_staticPlotting(); % Update Plot before Parsing RangeImage.
                
                r_img.findLineCandidates(); % (does what it says on the tin)
                
                
                pause(0.01); % Relief for Callback to Update
            end
            
            obj.close_lidar_staticPlotting(); % Object Found. Turn off Plotting.
            
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
        
        %% Get Trajectory Start
        % Determines the appropriate Starting Pose for a New Trajectory
        % based on the Previous Trajectory's End Pose and Whether the Robot
        % being Controlled is Using Map Localization.
        function p_s = getTrajectoryStart(obj)
            if(obj.rob.localizeAndFuse)
                % If Robot is using Map Localization, Assume it has the
                % most accurate estimate on its final world position.
                p_s = obj.rob.measTraj.p_f;
            else
                % Otherwise, start new trajectory where the last one ended.
                if(isempty(obj.start_poses))
                    p_s = obj.rob.init_pose; % First Run
                else 
                    p_s = obj.start_poses(end); % There was a Prev. Traj.
                end
            end % rob.localizeAndFuse?
        end
        
        %% Go To X (small)
        % Optionally: specify speed, spd
        function goTo_X_Small(obj, x_rel, spd)
            speed = 0.5*obj.rob.MAX_SPEED; % Default
            if(nargin > 2)
                speed = spd;
            end
            % Lower speed likely mean finer motion and less jerk -> lower
            % accels, .: scale with floor of 0.25*MAX_ACCEL.
            accel = 0.75*obj.rob.MAX_ACCEL * speed/obj.rob.MAX_SPEED + 0.25*obj.rob.MAX_ACCEL;
            
            %Anonymous function handle for velocity profile:
            vref = @(~,t)u_ref_trap(t,accel,speed,x_rel,0,0);
            omref = @(~,t)0;
            t_f = u_ref_trap(0,accel,speed,x_rel,0,1);
            
            initpose = obj.getTrajectoryStart();
            ttc = Trajectory_TimeCurve(vref,omref, 0, t_f, obj.traj_samples, initpose);
            
            if isempty(obj.feedback_controller)
                tf = Trajectory_Follower(obj.rob, ttc);
                obj.feedback_controller = tf.fbk_controller;
            else
                tf = Trajectory_Follower(obj.rob, ttc);
%                 tf = Trajectory_Follower(obj.rob, ttc, obj.feedback_controller);
%                 obj.feedback_controller.rt = ttc; % Super important to update this
            end
            
            tf.fbk_controller.correctiveTime = obj.k_tau;%* rt.getFinalTime();
            tf.fbk_trim = 0; % Turn off feedback trim for this in-exact motion. %%%%%%% ? ? %%%%%
               
            obj.time_loop(tf, 1);
            
            tf.fbk_trim = 1; % Reset
            
            % Until Mix-Ins are implemented for Trajectory Class Hierarchy,
            % Set traj_vec terminal TTC to equivalent TCS (to preserve
            % homogeneity).
%             eq_tcs = Trajectory_CubicSpiral.planTrajectory( ...
%                 x_rel, 0, 0, 1, ...
%                 obj.traj_samples, obj.tcs_scale ...
%             );
%             eq_tcs.init_pose = obj.traj_vec(end).getFinalPose();
%             eq_tcs.offsetInitPose();
%             obj.traj_vec(end+1) = eq_tcs;
            obj.traj_vec(end+1) = ttc;
            
            rel_pose = pose(x_rel,0,0);
            if (isempty(obj.start_poses))
                obj.start_poses = rel_pose;
            else
                obj.start_poses(end+1) = Trajectory.poseToWorld(rel_pose, obj.start_poses(end));
            end
            
            %Update plot after completed trajectory
            if(obj.plotting_enabled)
               obj.update_plot(tf);
            end
            
        end % #goTo_X_Small
        
        %% Go To Th (small)
        function goTo_th_Small(obj, th_rel)
            %Anonymous function handle for velocity profile:
            omref = @(~,t)u_ref_trap(t,3*obj.rob.MAX_ALPHA/4,0.5*obj.rob.MAX_OMEGA,th_rel,0,0);
            vref = @(~,t)0;
            t_f = u_ref_trap(0,3*obj.rob.MAX_ALPHA/4,0.5*obj.rob.MAX_OMEGA,th_rel,0,1);
            
            ttc = Trajectory_TimeCurve(vref,omref, 0, t_f, obj.traj_samples);
            
            initpose = obj.getTrajectoryStart();
            ttc.init_pose = initpose;%obj.traj_vec(end).getFinalPose();
            
            ttc.offsetInitPose();
            
            if isempty(obj.feedback_controller)
                tf = Trajectory_Follower(obj.rob, ttc);
                obj.feedback_controller = tf.fbk_controller;
            else
                tf = Trajectory_Follower(obj.rob, ttc);
%                 tf = Trajectory_Follower(obj.rob, ttc, obj.feedback_controller);
%                 obj.feedback_controller.rt = ttc; % Super important to update this
            end
            
            tf.fbk_controller.correctiveTime = obj.k_tau;%* rt.getFinalTime();
            tf.fbk_controller.isPureTurn = 1;
               
            obj.time_loop(tf, 1);
            
            tf.fbk_controller.isPureTurn = 0; % Reset
            
            % Until Mix-Ins are implemented for Trajectory Class Hierarchy,
            % Set traj_vec terminal TTC to equivalent TCS (to preserve
            % homogeneity).
%             eq_tcs = Trajectory_CubicSpiral([0 0 0], 3);
%             pf = obj.traj_vec(end).getFinalPose();
%             eq_tcs.init_pose = pose(pf.X, pf.Y, pf.th+th_rel);
%             eq_tcs.offsetInitPose();
%             obj.traj_vec(end+1) = eq_tcs;
            obj.traj_vec(end+1) = ttc;
            
            rel_pose = pose(0,0,th_rel);
            if (isempty(obj.start_poses))
                obj.start_poses = rel_pose;
            else
                obj.start_poses(end+1) = Trajectory.poseToWorld(rel_pose, obj.start_poses(end));
            end
            
            %Update plot after completed trajectory
            if(obj.plotting_enabled)
               obj.update_plot(tf);
            end
            
        end % #goTo_th_Small
        
        % Alias for goTo_th_Small
        function turn_stationary(obj, th)
            obj.goTo_th_Small(th);
        end % #turn_stationary
        
        %% Go To Absolute Position
        function goTo(obj,abs_pose)
            xa = abs_pose.x; ya = abs_pose.y; tha = abs_pose.th;
            
            rel_pose_vec = obj.rob.measTraj.p_f.aToB() * [xa; ya; 1];
            rel_pose_vec(3) = adel(tha, obj.rob.measTraj.p_f.th);
            
            obj.goTo_Rel( pose(rel_pose_vec) );
        end % #goTo
        
        %% Go To Relative Position
        function goTo_Rel(obj,rel_pose)
            x = rel_pose.x;
            y = rel_pose.y;
            th = rel_pose.th;
            
            rt = Trajectory_CubicSpiral.planTrajectory( ...
                x, y, th, 1, ...
                obj.traj_samples, obj.tcs_scale ...
            );
            initpose = obj.getTrajectoryStart();
            rt.init_pose = initpose;%obj.traj_vec(end).getFinalPose();
            % if you don't call offsetInitPose, Trajectory automatically
                %transforms each reference pose before handing it to mrpl
            rt.offsetInitPose();
            
            if isempty(obj.feedback_controller)
                tf = Trajectory_Follower(obj.rob, rt);
                obj.feedback_controller = tf.fbk_controller;
            else
                tf = Trajectory_Follower(obj.rob, rt);
%                 tf = Trajectory_Follower(obj.rob, rt, obj.feedback_controller);
%                 obj.feedback_controller.rt = rt; % Super important to update this
            end
            tf.fbk_controller.correctiveTime = obj.k_tau;%* rt.getFinalTime();
            
            obj.time_loop(tf, 1.75);
             
            %Store completed trajectory
            obj.traj_vec(end+1) = tf.rt;
            if (isempty(obj.start_poses))
                obj.start_poses = rel_pose;
            else
                obj.start_poses(end+1) = Trajectory.poseToWorld(rel_pose, obj.start_poses(end));
            end
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
%                 obj.clock.pause();
%                 obj.clock.resume(); % Break here for in-loop debugging
                
                if(obj.plotting_enabled)
                    obj.update_plotData(tf, T);
                end
                
                % Update the interface controller if there is one.
                if ~isempty(obj.interface)
                    % Read Interface Data:
                    obj.interface.readInterfaceInputs();

                    % Update Graphics:
                    obj.interface.update_graphics();
                end
                
                if(T > tf.rt.getFinalTime()+t_buffer)
                    done = 1;
                    obj.rob.moveAt(0,0); % Stop Immediately
                end
                
                pause(0.03);
            end % ~done?
            obj.rob.moveAt(0,0);
            %obj.rob.core.stop(); % Maybe throws things off?
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
            if isempty(obj.plot_figure) || ~isvalid(obj.plot_figure) || ~isgraphics(obj.plot_figure)
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
        
        %% Initialize Static Lidar Plotting
        % Plot Lidar Data when not Moving.
        function init_lidar_staticPlotting(obj)
            obj.laser_plotting_data.curr_fig = figure();
            grid on
            grid minor

            obj.laser_plotting_data.colorize = 1;
            if obj.laser_plotting_data.colorize
                obj.laser_plotting_data.lidar_plot = scatter(0,0,36,0);
            else
                obj.laser_plotting_data.lidar_plot = scatter(0,0,36);
            end
            set(gca, 'Xdir', 'reverse'); % Ensure Robot Y-Axis Points Left

            title({'LIDAR Data', '(Robot Reference Frame)'});
            xlabel('Y-Position [m]');
            ylabel('X-Position [m]');
            axis(1.2*[
                -RangeImage.MAX_RANGE RangeImage.MAX_RANGE ...
                -RangeImage.MAX_RANGE RangeImage.MAX_RANGE ...
            ]);
        end
        
        %% Update Lidar Plotting
        function update_lidar_staticPlotting(obj)
            if( obj.clock.time() - obj.laser_plotting_data.last_time > obj.laser_plotting_data.delay_width )
                r_img = obj.rob.hist_laser.last;
                figure(obj.laser_plotting_data.curr_fig);
                r_img.plot(obj.laser_plotting_data.colorize, obj.laser_plotting_data.lidar_plot);

                r_img.findLineCandidates();

                r_img.plotLineCandidates();

            obj.laser_plotting_data.last_time = obj.clock.time();
            end
        end
        
        %% Close Lidar Plotting
        function close_lidar_staticPlotting(obj)
            close(obj.laser_plotting_data.curr_fig);
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
            
            % ~"Slightly Underestimate Angle to produce less extreme
            % angles of approach"~ (use mechanical alignment to
            % correct):
            underest_scale = 0.04; %round down to nearest ~quarter degree (little less)
            th = underest_scale*floor(th1/underest_scale);
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
        acqPose = pose(x, y, th);
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - DATA MANIPULATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    end % mrplSystem <- methods(static)
    
end