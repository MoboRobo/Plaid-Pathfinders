% Full-Stack Teleop Interface for Controlling and Reading the Robot 
% (creates and manages its own mrplSystem).
classdef RobotInterface < handle
    %% PROPERTIES
    properties(Constant)
        % Base (sub-gain) Speeds for the Robot to Drive at.
        V_base = 0.02;
        om_base = 0.006; % 0.006/0.1
        
        NO_KEY = 0;             % Identifier for No Key being Pressed
    end % RobotKeyPressDriver <- properties(Constant)
    
    properties(GetAccess = public, SetAccess=private)
        mrpl; % MrplSystem Layer Responsible for Commanding Robot
        
        running = 0; % Whether the System is Currently Running (looping)
        loopFcns = struct( ... % Functions to Execute at Before and After Each Loop
            pre, ( @() 0 ), ... % Before
            post, ( @() 0 ) ... % After
        );
        
        fig_master; % Figure onto which all Information is Displayed
        
        keypress_data = struct( ...
            count, 0, ...       % How Many Times a Key has been Pressed (and logged)
            processed, 1, ...   % Whether the Most Recent Batch of Data has been Processed
            last_key, NO_KEY ...% Last Key Pressed
        );
        
        % Flags for what information to display (plotting):
        % (sum of all flags -> num. of subplots)
        displayPlottingFlags = struct( ...
            localization_plot, 1 ...    % What Robot Sees relative to Known Map
        );
    end % RobotKeyPressDriver <- properties(public,private)
    
    
    %% METHODS
    methods
        %% Constructor:
        function obj = RobotKeyPressDriver(robot_id, init_pose)
            obj.mrpl = mrplSystem( robot_id, init_pose );
            
            obj.fig_master = figure();
            set(obj.fig_master, 'KeyPressFcn', @obj.keyboardEventListener);
            
        end % Constructor
        
        %% Run:
        % Sets up and Runs System Loop t  Plot Relevant Information, and 
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

                obj.update_drive(1);

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
        
        %% Update Drive:
        % Updates the Drive Commands to the Robot based on the Recent
        % Keyboard Polls
        function update_drive(obj, vGain)
            V_max = obj.V_base*vGain;
            W = obj.mrpl.rob.WHEEL_TREAD;
            dV = obj.om_base * W * vGain;
            
            key = obj.pollKeyboard();
            
            if(key ~= obj.NO_KEY)
                if strcmp(key,'uparrow')
                    obj.mrpl.rob.sendVelocity(V_max, V_max);
                elseif strcmp(key,'downarrow')
                    obj.mrpl.rob.sendVelocity(-V_max, -V_max);
                elseif strcmp(key,'leftarrow')
                    obj.mrpl.rob.sendVelocity(V_max, V_max+dV);
                elseif strcmp(key,'rightarrow')
                    obj.mrpl.rob.sendVelocity(V_max+dV, V_max);
                elseif strcmp(key,'s')
                    disp('stop');
                    obj.mrpl.rob.sendVelocity(0.0, 0.0);
                    obj.mrpl.rob.core.stop();
                end
            end
            
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
        function keyboardEventListener(obj, ~, event)
            obj.keypress_data.count = obj.keypress_data.count + 1;
            obj.keypress_data.last_key = event.Key;
            obj.keypress_data.processed = 0;
        end % #keyboardEventListener
    end % RobotKeyPressDriver <- methods
end