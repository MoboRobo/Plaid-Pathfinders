% Full-Stack Teleop Interface for Controlling and Reading the Robot 
% (creates and manages its own mrplSystem).
classdef RobotInterface < handle
    %% PROPERTIES
    properties(Constant)
        % Base (sub-gain) Speeds for the Robot to Drive at.
        V_base = 0.02;
        om_base = 0.06; % 0.006/0.1
        
        NO_KEY = 0;             % Identifier for No Key being Pressed
    end % RobotInterface <- properties(Constant)
    
    properties(GetAccess = public, SetAccess=private)
        mrpl; % MrplSystem Layer Responsible for Commanding Robot
        
        rob_pose = pose(-15*0.0254,-9*0.0254,0);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Temp. Impl. until Sensor Fusion in P2_Robot
        
        vGain_default = 1; % Default Base Gain Applied to All Drive Velocities
        
        running = 0; % Whether the System is Currently Running (looping)
        loopFcns = struct( ... % Functions to Execute at Before and After Each Loop
            'pre', 0, ...   % Before
            'post', 0 ...   % After
        );
        
        fig_master; % Figure onto which all Information is Displayed
        fig_handles; % Handles Contained within the Interface Figure
        
        keypress_data = struct( ...
            'count', 0, ...         % How Many Times a Key has been Pressed (and logged)
            'processed', 1, ...     % Whether the Most Recent Batch of Data has been Processed
            'last_key', RobotInterface.NO_KEY ...  % Last Key Pressed
        );
        
        % Flags for what information to display (plotting):
        % (sum of all flags -> num. of subplots)
        displayPlottingFlags = struct( ...
            'odometry_plot', 1, ...     % Where the Robot has Moved since it Began
            'localization_plot', 1, ... % What Robot Sees relative to Known Map
            'lidar_feed', 1, ...        % Feed of Raw Range Images.
            'command_stream', 1 ...     % Stream of Commands Sent to Robot
        );
    
        % Handles for Plots
        % (default value must be some number ~=0,1)
        displayPlots = struct( ...
            'odometry', -1, ...     % Where the Robot has Moved since it Began
            ...
            'localization_image', -1, ... % What Robot Sees in World Coordinates
            'localization_world', -1, ... % Plot of Map in World Coordinates
            'localization_robot', -1, ... % Plot of Robot in World Coordinates
            ...
            'lidar_feed', -1 ...   % Feed of Raw Range Images.
        );
        
        % Meta Data for All Plots:
        displayMetaData = struct( ...
            'odometry_plot_maxFreq', 1, ... % Maximum Plotting Frequency of Odometry
            'odometry_plot_lastT', 0, ...   % Time of Last Plotting of Odometry
            ...
            'localization_plot_maxFreq', 1, ... % Maximum Plotting Frequency of Localization
            'localization_plot_lastT', 1, ...   % Time of Last Plotting of Localization
            ...
            'lidar_feed_maxFreq', 1, ...        % Maximum Plotting Frequency of Range Images
            'lidar_feed_density', 1, ...        % Fraction of Total RangeImage Points to Plot
            'lidar_feed_lastT', 1, ...          % Time of Last Plotting of the Lidar Feed
            ...
            'command_stream_count', 1, ...     % Total Number of Line Entries in Command Stream
            'command_stream_lastVelLen', 1, ...% Length of Robot CommVel Vector at Last Stream Update (this is not sufficient by itself because of data pre-allocation).
            'command_stream_lastVel', struct('V',0, 'om',0, 'v_l',0, 'v_r',0) ... % Velocity Profile (V,om,v_l,v_r) of Last Velocity Command
        );
    end % RobotInterface <- properties(public,private)
    
    
    %% METHODS
    methods
        %% Constructor:
        function obj = RobotInterface(robot_id, init_pose)
            obj.mrpl = mrplSystem( robot_id, init_pose );
            obj.rob_pose = init_pose;
            
            obj.fig_master = WorldFinder();
            obj.fig_handles = guidata(obj.fig_master);
            
            obj.readInterfaceInputs();
            set(obj.fig_master, 'KeyPressFcn', @obj.keyboardEventListener);
            
            obj.update_graphics(); % Update Interface Settings to Default Values (ie Slider Values)
            
        end % Constructor
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - TOP LEVEL
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% Run:
        % Sets up and Runs System Loop to Plot Relevant Information, and 
        % Control the Robot.
        % Optional: Calls the given Functions preFcn, postFcn at the
        % Beginning and End of Each Loop, Respectively.
        function run(obj, preFcn, postFcn)
            % One-Time Setup:
            if nargin > 1
                obj.loopFcns.pre = preFcn;
            end
            if nargin > 2
                obj.loopFcns.post = postFcn;
            end
            
            % Loop:
            obj.run_loop();
        end % # run
        
        % Actual Operational Loop Called by #run
        function run_loop(obj)
            obj.running = 1;
            while(obj.running)
            obj.loopFcns.pre();

                % Read Interface Data:
                obj.readInterfaceInputs();
            
                % Control Robot:
                obj.update_drive();
                
                % Update Graphics:
                obj.update_graphics();

            obj.loopFcns.post();
            pause(0.01); % CPU Relief
            end
        end % #run_loop
        
        % Pauses/Suspends the Run Loop:
        function pause(obj)
            obj.running = 0;
        end % #pause
        % Resumes the Run Loop:
        function resume(obj)
            obj.run_loop();
        end
        
        % Stops Running the Loop and Closes any Necessary Processes:
        function stop(obj)
            obj.pause();
        end% #stop
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - GRAPHICS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% Update Graphics
        % Updates all graphics in the interface.
        function update_graphics(obj)
            obj.update_odometryPlot();
            obj.update_localizationPlot();
            obj.update_lidarFeed();
            obj.update_commandStream();
        end
        
        function update_odometryPlot(obj)
            set(obj.fig_handles.RobotOdometryPlotMaxFreqText,'String',obj.displayMetaData.odometry_plot_maxFreq);
            
            if(obj.displayPlottingFlags.odometry_plot)
                if( (obj.mrpl.clock.time() - obj.displayMetaData.odometry_plot_lastT) > 1/obj.displayMetaData.odometry_plot_maxFreq )
                    as = obj.fig_handles.RobotOdometryAxes;
                    
                    xs = obj.mrpl.rob.measTraj.xs;
                    ys = obj.mrpl.rob.measTraj.ys;
                    
                    obj.displayPlots.odometry = plot(as, ys,xs);
                    if(~strcmp(get(as,'XDir'),'reverse'))
                        set(as, 'Xdir', 'reverse'); % Ensure Robot Y-Axis Points Left
                    end
                    
                    obj.displayMetaData.odometry_plot_lastT = obj.mrpl.clock.time();
                end % Dt>1/fmax
            end % if plotting on
        end
        
        function update_localizationPlot(obj)
            set(obj.fig_handles.MapLocalizationPlotMaxFreqText,'String',obj.displayMetaData.localization_plot_maxFreq);
            
            if(~obj.mrpl.rob.laser_state) % If lasers are off
                obj.mrpl.rob.laserOn(); % Turn Lasers on
                pause(1); % Wait for Lasers to generate RangeImages.
            end
            
            r_img = obj.mrpl.rob.hist_laser.last;

            % To get every nth RangeImage Point, n is spec_vol, the
            % Specific Volume:
            spec_vol = 10;

%                     rngs = r_img.raw(1:spec_vol:end);
%                     angs = r_img.raw_ang(1:spec_vol:end);
%                     [xs, ys] = RangeImage.arToXy(rngs, angs);
%                     rangePts = [xs; ys; ones(size(xs))];

            rngs = r_img.data.ranges(1:spec_vol:end);
            xs = r_img.data.xs(1:spec_vol:end);
            ys = r_img.data.ys(1:spec_vol:end);
            rangePts = [xs; ys; ones(size(xs))];

            len_wall = 2;
            World_Map = [0 0 -len_wall; -len_wall 0 0; 1 1 1];

            % Get Localized Current Pose of Robot
            [~, curPose, ptsAnalysed] = Lab10_WorldLocalize(World_Map, rangePts, obj.rob_pose); %%%%%%%%%%%%%%%%%% TODO: CHANGE THIS IN FUTURE.

            rangePts = ptsAnalysed; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% YO, DAWG, I CHANGED THIS
            % Transform RangeImage into World Coordinates:
            worldPts = curPose.bToA()*rangePts;
            xs = worldPts(1,:);
            ys = worldPts(2,:);

            % Transform Robot Body Points into World Coordinates:
            robPts = robotKinematicModel.bodyGraph();
            robPts = curPose.bToA()*robPts;

            % Update Robot Pose Estimate:
            obj.rob_pose = curPose;
            
            
            if(obj.displayPlottingFlags.localization_plot || 1)
                if( (obj.mrpl.clock.time() - obj.displayMetaData.localization_plot_lastT) > 1/obj.displayMetaData.localization_plot_maxFreq )
                    
                    as = obj.fig_handles.MapLocalizationAxes;
                    
                    if ~sum(isnan(rngs))
                        if ~isgraphics(obj.displayPlots.localization_image) % Initialize Plot if Not Yet Instantiated
                            axes(as);
%                             figure();
                            hold on
                                obj.displayPlots.localization_image = scatter(ys, xs, 36, 'r');
                                obj.displayPlots.localization_world = plot(World_Map(2,:),World_Map(1,:), 'b');
                                obj.displayPlots.localization_robot = plot(robPts(2,:),robPts(1,:), 'g');
                                axis equal
                            hold off
%                             set('Xdir', 'reverse'); % Ensure Robot Y-Axis Points Left
                            set(as, 'Xdir', 'reverse'); % Ensure Robot Y-Axis Points Left
                        end
                        hold on
                            set(obj.displayPlots.localization_image, 'XData', ys, 'YData', xs);
                            set(obj.displayPlots.localization_robot, 'XData', robPts(2,:), 'YData', robPts(1,:));
                        hold off
                    else
                        warning('No Data in Range to Plot or Invalid Range Data');
                    end
                    
                    if(~strcmp(get(as,'XDir'),'reverse'))
                        set(as, 'Xdir', 'reverse'); % Ensure Robot Y-Axis Points Left
                    end
                    
                    obj.displayMetaData.localization_plot_lastT = obj.mrpl.clock.time();
                end % Dt>1/fmax
            end % if plotting on
        end
        
        function update_lidarFeed(obj)
            if(obj.displayPlottingFlags.lidar_feed)
                if( (obj.mrpl.clock.time() - obj.displayMetaData.lidar_feed_lastT) > 1/obj.displayMetaData.lidar_feed_maxFreq )
                    
                    if(~obj.mrpl.rob.laser_state) % If lasers are off
                        obj.mrpl.rob.laserOn(); % Turn Lasers on
                        pause(1); % Wait for Lasers to generate RangeImages.
                    end
                    
                    as = obj.fig_handles.LidarFeedAxes;
                    
                    r_img = obj.mrpl.rob.hist_laser.last;
                    
                    % To get every nth RangeImage Point, n is spec_vol, the
                    % Specific Volume:
                    spec_vol = 1 / obj.displayMetaData.lidar_feed_density;
                    
                    rngs = r_img.raw(1:spec_vol:end);
                    angs = r_img.raw_ang(1:spec_vol:end);
                    
                    [xs, ys] = RangeImage.arToXy(rngs, angs);
                    
                    if ~sum(isnan(rngs))
                        if ~isgraphics(obj.displayPlots.lidar_feed) % Initialize Plot if Not Yet Instantiated
                            obj.displayPlots.lidar_feed = scatter(as, ys, xs, 36, rngs);
                            set(as, 'Xdir', 'reverse'); % Ensure Robot Y-Axis Points Left
                        end

                        set(obj.displayPlots.lidar_feed, 'XData', ys, 'YData', xs, 'CData', rngs);
                    else
                        warning('No Data in Range to Plot or Invalid Range Data');
                    end
                    
                    if(~strcmp(get(as,'XDir'),'reverse'))
                        set(as, 'Xdir', 'reverse'); % Ensure Robot Y-Axis (still) Points Left
                    end
                    
                    obj.displayMetaData.lidar_feed_lastT = obj.mrpl.clock.time();
                end % Dt>1/fmax
            end % if plotting on
        end
        
        function update_commandStream(obj)
            
            % UPDATE IF THERE HAS BEEN A VELOCITY COMMAND.
            len = obj.mrpl.rob.commTraj.data_V.numElements;
            V_curr = obj.mrpl.rob.commTraj.V_f;
            om_curr = obj.mrpl.rob.commTraj.om_f;
            cwv_curr = obj.mrpl.rob.hist_commWheelVel.last;
            v_l_curr = cwv_curr.v_l;
            v_r_curr = cwv_curr.v_r;
            
            last_vp = obj.displayMetaData.command_stream_lastVel;
            if( ...
            obj.displayMetaData.command_stream_lastVelLen < len ...
            || last_vp.V ~= V_curr ...
            || last_vp.om ~= om_curr ...
            || last_vp.v_l ~= v_l_curr ...
            || last_vp.v_r ~= v_r_curr ...
            )
                obj.displayMetaData.command_stream_lastVelLen = len;
                obj.displayMetaData.command_stream_lastVel = struct(...
                    'V', V_curr, ...
                    'om', om_curr, ...
                    'v_l', v_l_curr, ...
                    'v_r', v_r_curr ...
                );
                
                obj.addCommandStreamEntry(strcat( ...
                    'Profile: ', ...
                    'V: ', num2str(V_curr, 3), ...
                    ', ', ...
                    '\omega: ', num2str(om_curr, 3), ...
                    ' -> ', ...
                    'L: ', num2str(v_l_curr, 3), ...
                    ', ', ...
                    'R: ', num2str(v_r_curr, 3) ...
                ));
                    
            end
        end
        
        %% Add Command Stream Entry
        % Adds a new line entry, str, to the command stream.
        function addCommandStreamEntry(obj, str)
            list_old = get(obj.fig_handles.CommandStreamList,'string');
            set(obj.fig_handles.CommandStreamList,'string',[list_old; cellstr(str)]);
            
            obj.displayMetaData.command_stream_count = obj.displayMetaData.command_stream_count + 1;
            set(obj.fig_handles.CommandStreamList,'value',obj.displayMetaData.command_stream_count);
        end % #addCommandStreamEntry
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - INTERFACING
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% Read Interface Inputs:
        % Read User Input Data from the Interface (do this in one function
        % so that switching/updating interfaces is easy).
        function readInterfaceInputs(obj)
            obj.displayPlottingFlags.odometry_plot = get(obj.fig_handles.RobotOdometryPlotState,'value');
            obj.displayPlottingFlags.localization_plot = get(obj.fig_handles.MapLocalizationPlotState,'value');
            obj.displayPlottingFlags.lidar_feed = get(obj.fig_handles.LidarFeedState,'value');
            obj.displayPlottingFlags.command_stream = get(obj.fig_handles.CommandStreamState,'value');
    
            obj.displayMetaData.odometry_plot_maxFreq = get(obj.fig_handles.RobotOdometryPlotMaxFreq,'value');
            obj.displayMetaData.localization_plot_maxFreq = get(obj.fig_handles.MapLocalizationPlotMaxFreq,'value');
            obj.displayMetaData.lidar_feed_maxFreq = get(obj.fig_handles.LidarFeedMaxFreq,'value');
            obj.displayMetaData.lidar_feed_density = get(obj.fig_handles.LidarFeedDensity,'value');
        end
        
        %% Poll Keyboard:
        % Processes Keypress Data and Returns the Most Recent Key Presssed
        function key = pollKeyboard(obj)
            if(~obj.keypress_data.processed)
                key = obj.keypress_data.last_key;
                obj.keypress_data.processed = 1;
            else
                key = obj.NO_KEY; % No Key Pressed since Last Poll
            end
        end % #pollKeyboard
        
        %% Keyboard Event Listener:
        % Callback Function for the Keyboard
        function keyboardEventListener(obj, ~, event)
            obj.keypress_data.count = obj.keypress_data.count + 1;
            obj.keypress_data.last_key = event.Key;
            obj.keypress_data.processed = 0;
        end % #keyboardEventListener
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - MOTION CONTROL
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% Update Drive:
        % Updates the Drive Commands to the Robot based on the Recent
        % Keyboard Polls
        % Optionally: specify gain to send layer onto all velocities, vGain
        function update_drive(obj, vGain)
            k_V = obj.vGain_default;
            if nargin>1
                k_V = vGain;
            end
            
            V_max = obj.V_base*k_V;
            W = obj.mrpl.rob.WHEEL_TREAD;
            dV = obj.om_base * W * k_V;
            
            key = obj.pollKeyboard();
            
            if(key ~= obj.NO_KEY)
                if strcmp(key,'uparrow')
                    obj.addCommandStreamEntry('> Fwd.');
                    obj.mrpl.rob.sendVelocity(V_max, V_max);
                elseif strcmp(key,'downarrow')
                    obj.addCommandStreamEntry('> Rev.');
                    obj.mrpl.rob.sendVelocity(-V_max, -V_max);
                elseif strcmp(key,'leftarrow')
                    obj.addCommandStreamEntry('> Fwd L Turn.');
                    obj.mrpl.rob.sendVelocity(V_max, V_max+dV);
                elseif strcmp(key,'rightarrow')
                    obj.addCommandStreamEntry('> Fwd R Turn.');
                    obj.mrpl.rob.sendVelocity(V_max+dV, V_max);
                elseif strcmp(key,'s')
                    obj.addCommandStreamEntry('> Stop.');
                    disp('stop');
                    obj.mrpl.rob.sendVelocity(0.0, 0.0);
                    obj.mrpl.rob.core.stop();
                end
            end
        end % #update_drive
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - PERCEPTION CONTROL
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% Take Image
        % Takes an Image with the Robot's Camera and Displays it.
        function takeImage(obj)
            obj.mrpl.rob.core.captureImage();
        end % #takeImage
            
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - OUTPUT CONTROL
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% Say:
        % Uses TTL to Make the Robot Say the Given String of Text, str.
        function say(obj, str)
            obj.mrpl.rob.core.say(str);
        end % #say
    end % RobotInterface <- methods
end