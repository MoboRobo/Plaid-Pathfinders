% A Wrapper for the RaspBot/Neato Robot API Class
% We might be glad we made this later if we need one so we don't have to
% our other API functions/classes to include it later on.
% (N.B. Large methods of the class stored in @P2_Robot class folder)
%
% A note on coordinate system: +ve x points forward out of robot, +ve y
% points to the left, th (bearing) ranges from 0 at +ve x axis, going CCW
% to pi at -ve x axis and CW to -pi.
%                     +x
%                     |
%                     |
%                   0 | 0
%           +y -------+-------
%                     |
%                     |
%                 +pi | -pi
classdef P2_Robot < handle
    %% PROPERTIES
    properties (GetAccess=public, SetAccess=private)
        core;     	% RaspBot Core Class (manages ROS communication, et al)
        
        on_time; 	% Time (tic) that the robot started
        getTime;    % Lambda Function for Getting Time
                    % (for opt. use of external clocks)
        
        
        %Motion:
        curr_V = 0;     % m/s, Most Recently Commanded Body Velocity
        curr_omega = 0; % rad/s, Most Recently Commanded Rotational Velocity
        
        %Raspbot Data Logs:
        
        % History of Encoder Readings [m]
        % Formatted as: [[sl_0,sr_0, t_0]; ... ]:
        hist_enc = slidingFifo(10000, struct('s_l',0, 's_r',0, 't',0));
        % History of Robot Wheel Velocities, determined from Sensor Readings.
        % Formatted as: [[vl_0,vr_0]; ... ]
        hist_estWheelVel = slidingFifo(10000, struct('v_l',0, 'v_r',0));
        % Robot Trajectory determined by Encoders (Odometry), RobotTrajectory:
        encTraj;
        
        % History of Velocity Commands Issued to the Robot:
        % Formatted as: [[vl_0,vr_0,V,0,omega,0,t_0]; ... ]
        hist_commWheelVel = slidingFifo(10000, struct('v_l',0, 'v_r',0));
        % Robot Commanded Trajectory (Odometry), RobotTrajectory:
        commTraj;
        
        % History of lidar laser RangeImages.
        hist_laser = slidingFifo(100, RangeImage(zeros(1,360)));
        % Whether the lidar laser is on.
        laser_state = 0;
        
        % Measured Trajectory, Points to Handle of whichever Trajectory
        % (Encoder, Lidar, Fusion, etc.) should represent the Robot's
        % official "measured" trajectory.
        measTraj;
        

        %Odometry:
        trip_startTime; % Time Most Recent Trip Started (odometry)
        init_enc_l;     % Left Encoder Reading from Start of Trip
        init_enc_r;     % Left Encoder Reading from Start of Trip
        
        %Plotting
        position_plotting_on = 0;   % bool, Whether Position Plotting is Enabled
        position_plotting_listener; % Handle for the Last Listener to the Position Plotting Event.
        %Resources for the Position Plot:
        position_plot_resources = struct( ...
            'fig',nan, ...      % figure handle
            'Comm_Plot',nan, ...% plot of commanded positions
            'Est_Plot',nan, ... % plot of estimated positions
            'setup',0 ...       % Whether setup has been performed
        );
        % Properties of the Position Plot:
        position_plot_properties = struct( ...
            'period', 1/3, ...              % s, Minimum amount of time between plots
            'T_last', 0, ...                % s, Time of last plot in robot time, toc(on_time)
            'title', 'Robot Position' ...   % Title of the Plot
        );
        
    end % P2_Robot->properties(public,private)
    
    properties (Constant)
        %Geometry:
        WHEEL_TREAD = 0.08675; % [m] Lateral Distance between Wheel Centers
        
        %Motion:
        L2R_RATIO = 1;  % Ratio of Natural Left-to-Right Wheel Running Speeds (Curvature Correction)
        MIN_SPEED = 0.012; % m/s, Minimum Sustainable Wheel Velocity below
                           % which the robot cannot move (due to static 
                           % friction and rpm/torque curve)
                           
        MAX_SPEED = 0.5;% m/s, Maximum Linear Speed of Robot
        MAX_ACCEL = 0.75;% m/s^2, Maximum Linear Acceleration of Robot
        MAX_OMEGA = 8;%   rad/s, Maximum Rotational Speed of Robot.
        MAX_ALPHA = 5;%   rad/s^2, Maximum Angular Acceleration of Robot
    end % P2_Robot->properties(public,public)
    
    %% EVENTS
    % Events to Listen For
    events
        P2_PLOTING_POSITION
    end
    
    methods
        %% Constructor
        % rb - RaspBot/Neato Robot Class which manages ROS Communication
        function obj = P2_Robot(rb, time_lambda)
            if nargin>0
                if isa(rb,'raspbot')
                    obj.core = rb;
                else
                    error('Debugger Robot must be a raspbot')
                end % r is raspbot?
                
            else
                error('Must give Debugger a Robot to track')
            end % nargin>0?
            
          % Initialize Timing:
            obj.on_time = tic;
            obj.trip_startTime = tic;
            obj.startTrip(); % Default odometry starts at instantiation.
            if nargin>1
                obj.getTime = time_lambda;
            else
                obj.getTime = @()toc(obj.on_time);
            end % nargin>1
            
            obj.waitForReady();
            evnt = obj.core.encoders.LatestMessage;
            obj.hist_enc.que(1) = struct('s_l',evnt.Vector.X, 's_r',evnt.Vector.Y, 't',(evnt.Header.Stamp.Sec + evnt.Header.Stamp.Nsec/1e9));
            
            obj.encTraj = RobotTrajectory();
            obj.commTraj = RobotTrajectory();
            
            obj.measTraj = obj.encTraj;
            
            obj.init_enc_l = obj.core.encoders.LatestMessage.Vector.X;
            obj.init_enc_r = obj.core.encoders.LatestMessage.Vector.Y;
            
            % Establish Data-Logging and Odometry Callback Functions
            obj.logEncoders();
            obj.processNewEncoderData(obj.core.encoders, evnt);
            
            %Wait for Sim to Initialize:
            pause(1.5);
            obj.core.sendVelocity(0,0);
            pause(1);
        end % #P2_Robot Constructor
        
        %% Destructor
        function delete(obj)
%             obj.core.delete(); %Destruct Core Robot
        end % #delete
        
        %% INITIALIZATION
        % Methods relating to the setup process of the robot.
        
        % Waits for the Robot to Properly Initialize
        function waitForReady(obj)
            while(~obj.ready()); end
            pause(1.5); % Extra Assurance. Sometimes the motion timing acts
                        % inconsistently when plot have been initialized
                        % and there isn't a wait of at least 1 sec.
                        % afterwards.
        end % #waitForReady
        
        % Returns Whether the Robot has Completed all Initialization
        % Procedures and is Ready to Run.
        function r = ready(obj)
            r = 1;
            r = r & obj.plotsReady();
        end % #ready
        
        
        %% KINEMATICS
        % Transforming between sensor readings, robot commands, and the
        % world using WMR kinematics.
        
        % Calculates the Rigid Body Rotational Velocity and Body-Center
        % Velocity of the Robot from its Left and Right Wheel Speeds.
        % (This method can also be fed a vector of v_l and of v_r.
        function [V, om] = computeIK(obj, v_l, v_r)
            V = (v_r+v_l) ./ 2; 
            om = (v_r-v_l) ./ obj.WHEEL_TREAD;
        end % #computeIK
        
        %% SENSING
        function processNewLaserData(obj, ~, event)
            obj.hist_laser.add(RangeImage(event.Ranges));
     
            
        end
        function laserOn(obj)
            if ~obj.laser_state
                %Reset Lidar History:
                obj.hist_laser = slidingFifo( ...
                    obj.hist_laser.maxElements, ...
                    RangeImage(zeros(1,360)) ...
                );
                
                obj.core.startLaser();
                obj.core.laser.NewMessageFcn = @obj.processNewLaserData;
                obj.laser_state = 1;
            
            end
        end
        function laserOff(obj)
            if obj.laser_state
                obj.laser_state = 0;
                obj.core.stopLaser();
            end
        end
        
        %% ODOMETRY
        % Basic position tracking (time and encoder deltas, etc.)
        
        % Turn on Encoder Data Logging and Odometry Calculation
        function logEncoders(obj)
            obj.core.encoders.NewMessageFcn = @obj.processNewEncoderData;
        end % #logEncoders
        
        % Resets All Robot State Data
        function resetStateData(obj)
            obj.hist_enc.add(obj.hist_enc.first());
            obj.hist_estWheelVel.add(obj.hist_estWheelVel.first());
            obj.encTraj.reset();
            
            obj.hist_commWheelVel.add(obj.hist_commWheelVel.first());
            obj.commTraj.reset();
            
            obj.hist_laser = slidingFifo(100, RangeImage(zeros(1,360)));
            
            obj.curr_V = 0;
            obj.curr_omega = 0;
            
            obj.on_time = tic;
        end
        
        % Processes New Encoder Data by Storing it and Computing Velocity
        % and Dead-Reckiong a Body Position.
        function processNewEncoderData(obj, ~, event)
            %Immediately Capture Event Data:
            s_l = event.Vector.X;
            s_r = event.Vector.Y;
            t = event.Header.Stamp.Sec + event.Header.Stamp.Nsec/1e9;
            
            %Compute Amount of Time Elapsed since Previous Measurement
            dt = t - obj.hist_enc.last.t;
            
            if(dt > 0)
                %Compute Translational Velocity of the Robot's since the last
                %Measurement:
                v_l = (s_l - obj.hist_enc.last.s_l) / dt;
                v_r = (s_r - obj.hist_enc.last.s_r) / dt;

                %Now that Previous Command is Done (a new command has been issued),
                %compute IK and robot pose based on how long it was active for.
                [V, omega] = obj.computeIK(v_l, v_r);
                
                obj.encTraj.update(V,omega,t);
                
                obj.hist_estWheelVel.add(struct('v_l',v_l, 'v_r',v_r));
                
                obj.triggerPositionPlot();
                
            end % dt>0?
            
            obj.hist_enc.add(struct('s_l',s_l, 's_r',s_r, 't',t));
        end % #processNewEncoderData
        
        % DEPRECATED
        % Sets/Saves a new Robot State from which Odometry is Collected (overwrites 
        % previous).
        % Returns vector containing new state variables.
        state = startTrip(obj)
        
        % DEPRECATED
        % Time Elapsed since Start of Trip
        function t = tripTime(obj)
            t = toc(obj.trip_startTime);
        end
        
        % DEPRECATED
        % Change in Left Encoder Distance since Start of Trip
        function d = leftTripDist(obj)
            d = obj.core.encoders.LatestMessage.Vector.X - obj.init_enc_l;
        end
        % DEPRECATED
        % Change in Right Encoder Distance since Start of Trip
        function d = rightTripDist(obj)
            d = obj.core.encoders.LatestMessage.Vector.Y - obj.init_enc_r;
        end % #rightTripDist
        % DEPRECATED
        % Change in Average Encoder Distance since Start of Trip
        function d = avgTripDist(obj)
            d = (obj.leftTripDist() + obj.rightTripDist()) / 2;
        end % #rightTripDist
        
        %% MOTION
        % Methods for going places
        
        % Move the Robot with Left and Right Wheel Speeds v_l and v_r,
        % Logs the Commands into the Command History hist_commVel, and
        % Computes and Logs the Commanded Pose into hist_commPose.
        % (wrapper for the RaspBot setVelocity Command.
        sendVelocity(obj, v_l, v_r);
        
        % Moves the Robot with a Speed, V (as measured from robot center) and
        % Rotational Velocity, omega.
        moveAt(obj, V, omega);
        
        % Calculates a Constant Curvature Trajectory to the Point at
        % (x,y) relative to the robot, with bearing th, and then moves along it at 
        % velocity, V. [See the note in the P2_Robot Class for coordinate 
        % standards].
        trajectory_goTo(obj, V, x, y, th);
        
        
        %% PLOTTING
        % Methods for Plotting and Displaying Information about the Robot's
        % State.
        
        % Triggers a PositionPlotting Event
        function triggerPositionPlot(obj)
            notify(obj, 'P2_PLOTING_POSITION');
        end % #triggerPlot
        % Registers a Listener for the PositionPlotting Event
        function lh = addPositionPlotListener(obj, listener)
             lh = addlistener(obj,'P2_PLOTING_POSITION', listener);
        end % #addPositionPlotListener
        
        % Enables Position Plotting for the Robot
        function enablePositionPlotting(obj)
            obj.position_plotting_on = 1;
            obj.position_plotting_listener = obj.addPositionPlotListener(@obj.plotPosition);
            if(~obj.position_plot_resources.setup)
                obj.setupPositionPlot();
            end
        end % #enablePositionPlotting
        % Disables Position Plotting for the Robot
        function disablePositionPlotting(obj)
            obj.position_plotting_on = 0;
            delete(obj.position_plotting_listener);
        end % #disablePositionPlotting
        
        
        %Setup the Position Plot
        function setupPositionPlot(obj)
            if strcmp(obj.core.name, 'sim')
                while(~obj.core.sim_robot.started); end
            end % core.name == 'sim'?
            obj.position_plot_resources.fig = figure();
            hold on
                obj.position_plot_resources.Comm_Plot = plot(0, 0, 'k-'); % Plot of Commanded Dead-Reckoning Positions ( int dX.(X,u) )
                obj.position_plot_resources.Est_Plot = plot(0, 0, 'b');  % Plot of Measured/Estimated Dead-Reckoning Positions ( int dX.(X,z) )
            hold off
            axis equal
            legend('Position Reckoned from Commands', 'Position Estimated from Readings');
            title(obj.position_plot_properties.title);
            drawnow
            while(~strcmp(obj.position_plot_resources.fig.CurrentAxes.Title.String,obj.position_plot_properties.title)); end % Ensure Updatesare Finished
            
            obj.position_plot_resources.setup = 1; %Setup Complete
        end % #setupPositionPlot
        %Plot the Position of the Data of the Robot
        function plotPosition(obj, ~,~)
            tt = toc(obj.on_time);
            Dt = tt - obj.position_plot_properties.T_last;
            
            if(obj.position_plotting_on ...
            && obj.position_plot_resources.setup ...
            && Dt > obj.position_plot_properties.period)
        
                figure(obj.position_plot_resources.fig);
                hold on
                    xxs = [obj.hist_commPose(:).poseVec]; xxs = xxs(1,:);
                    yys = [obj.hist_commPose(:).poseVec]; yys = yys(2,:);
                    set(obj.position_plot_resources.Comm_Plot, ...
                        'xdata',-yys, ...
                        'ydata',xxs ...
                    );
                    xxs = [obj.hist_estPose(:).poseVec]; xxs = xxs(1,:);
                    yys = [obj.hist_estPose(:).poseVec]; yys = yys(2,:);
                    set(obj.position_plot_resources.Est_Plot, ...
                        'xdata',-yys, ...
                        'ydata',xxs ...
                    );
                hold off
                obj.position_plot_properties.T_last = tt;
                
            end % if
        end % #plotPosition
        
        
        % Returns a Boolean about Whether all the Enabled Plots are Ready.
        function r = plotsReady(obj)
            r = 1;
            if(obj.position_plotting_on)
                r = r & strcmp(obj.position_plot_resources.fig.CurrentAxes.Title.String,obj.position_plot_properties.title);
            end % position_plotting_on?
        end % #plotsReady
    end % P2_Robot->methods
    
end % P2_Robot