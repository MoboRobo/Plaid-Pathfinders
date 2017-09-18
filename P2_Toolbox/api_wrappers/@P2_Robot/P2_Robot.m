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
        core;           % RaspBot Core Class (manages ROS communication, et al)
        on_time;        % Time (tic) that the debugger started
        
        %Motion:
        curr_V = 0;     % m/s, Most Recently Commanded Body Velocity
        curr_omega = 0; % rad/s, Most Recently Commanded Rotational Velocity
        
        %Raspbot Data Logs:
        hist_enc_l=[];  % m, History of Left Encoder Data
        hist_enc_r=[];  % m, History of Right Encoder Data
        hist_enc_t=[];  % s, History of Encoder Call Times
        hist_enc_currFrame = 1;
        
        hist_laser=[];  % m, History of LIDAR data
        hist_laser_t=[];  % s, History of Encoder Call Times
        
        %Odometry Data Logs:
        % (store lots of calculated values as storage is a prevalent and
        % CPU time is highly valueable)
        hist_bodyV=[0];  % m/s, History of Rigid Body Velocity at Center
        hist_bodyOmega=[0];% rad/s, History of Rigid Body Rotation Velocity
        hist_x=[0];  % m, History of X Position in the World-Frame
        hist_y=[0];  % m, History of Y Position in the World-Frame
        hist_th=[0];  % m, History of Heading, th, in the World-Frame
        hist_currFrame = 2;
        
        %Odometry:
        trip_startTime; % Time Most Recent Trip Started (odometry)
        init_enc_l;     % Left Encoder Reading from Start of Trip
        init_enc_r;     % Left Encoder Reading from Start of Trip
    end % P2_Robot->properties(public,private)
    
    properties (GetAccess=public, SetAccess=public)
        %Geometry:
        WHEEL_TREAD = 0.09; % [m] Lateral Distance between Wheel Centers
        
        %Motion:
        L2R_RATIO = 1;  % Ratio of Natural Left-to-Right Wheel Running Speeds (Curvature Correction)
        MIN_SPEED = 0.012; % m/s, Minimum Sustainable Wheel Velocity below
                           % which the robot cannot move (due to static 
                           % friction and rpm/torque curve)
    end % P2_Robot->properties(public,public)
    
    methods
        %% Constructor
        % rb - RaspBot/Neato Robot Class which manages ROS Communication
        function obj = P2_Robot(rb)
            if nargin>0
                if isa(rb,'raspbot')
                    obj.core = rb;
                else
                    error('Debugger Robot must be a raspbot')
                end % r is raspbot?
            else
                error('Must give Debugger a Robot to track')
            end % nargin>0?
            
            % Establish Data-Logging and Odometry Callback Functions
            %obj.core.encoders.NewMessageFcn = @processNewEncoderData;
            
            obj.on_time = tic;
            obj.trip_startTime = tic;
            obj.init_enc_l = obj.core.encoders.LatestMessage.Vector.X;
            obj.init_enc_r = obj.core.encoders.LatestMessage.Vector.Y;
            obj.startTrip(); % Default odometry starts at instantiation.
        end % #P2_Robot Constructor
        
        %% Destructor
        function delete(obj)
%             obj.core.delete(); %Destruct Core Robot
        end % #delete
        
        %% ODOMETRY
        % Basic position tracking (time and encoder deltas, etc.)
        
        % Sets/Saves a new Robot State from which Odometry is Collected (overwrites 
        % previous).
        % Returns vector containing new state variables.
        state = startTrip(obj)
        
        % Time Elapsed since Start of Trip
        function t = tripTime(obj)
            t = toc(obj.trip_startTime);
        end
        
        % Change in Left Encoder Distance since Start of Trip
        function d = leftTripDist(obj)
            d = obj.core.encoders.LatestMessage.Vector.X - obj.init_enc_l;
        end
        % Change in Right Encoder Distance since Start of Trip
        function d = rightTripDist(obj)
            d = obj.core.encoders.LatestMessage.Vector.Y - obj.init_enc_r;
        end % #rightTripDist
        % Change in Average Encoder Distance since Start of Trip
        function d = avgTripDist(obj)
            d = (obj.leftTripDist() + obj.rightTripDist()) / 2;
        end % #rightTripDist
        
        %% MOTION
        % Methods for going places
        
        %Run Robot with relative wheel velocities v_l, v_r, correcting for
        %curvature defined by L2R_Factor (Ratio of Left-to-Right Wheel Running
        %Speeds under natural conditions due to mechanical and
        %environmental factors such as axel friction and tire deformation).
        function run_correct(obj, v_l, v_r)
            obj.core.sendVelocity(v_l/obj.L2R_RATIO,v_r)
        end % #runStraight
        
        % Moves the Robot with a Speed, V (as measured from robot center) and
        % Rotational Velocity, omega.
        % sim - [bool] Whether the robot is simulated. If it is, the sendVelocity
        % command takes (v_left,v_right); if real, it takes (v_right,v_left).
        moveAt(obj, V, omega, sim);
        
        % Calculates a Constant Curvature Trajectory to the Point at
        % (x,y) relative to the robot, with bearing th, and then moves along it at 
        % velocity, V. [See the note in the P2_Robot Class for coordinate 
        % standards].
        trajectory_goTo(obj, V, x, y, th);
        
%         % Process New Encoder Data Sent from the RaspBot by logging it and
%         % calculating the odometry parameters from it.
%         function processNewEncoderData(obj, handle, event)
%             sl = event.Vector.X;
%             sr = event.Vector.Y;
%             obj.hist_enc_l(end+1) = sl;
%             obj.hist_enc_r(end+1) = sr;
%             
%             vl = (sl - 
%             
%             obj.hist_bodyV(end+1) = 
%             obj.hist_bodyOmega(end+1) =
%             
%             obj.hist_th(end+1) = 
%             
%             ds = 
%             dth = 
%             dV = 
%             dOmega = 
% 
%             
%         end % #processNewEncoderData
         function encoderEventListener(obj,handle,event)
             encoderXMessage = event.Vector.X;
             encoderYMessage = event.Vector.Y;
             encoderDataTimestamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1000000000.0;
             
             obj.hist_enc_l(obj.hist_enc_currFrame) = encoderXMessage;
             obj.hist_enc_r(obj.hist_enc_currFrame) = encoderYMessage;
          
             if(obj.hist_enc_currFrame > 1)
                obj.hist_enc_t(obj.hist_enc_currFrame) = encoderDataTimestamp+obj.hist_enc_t(obj.hist_enc_currFrame-1);
             else
                obj.hist_enc_t(obj.hist_enc_currFrame) = encoderDataTimestamp; 
             end
             
             dt = encoderDataTimestamp; %dt in seconds
             vr = 0;
             vl = 0;
             if(obj.hist_enc_currFrame > 1)
                vr = (obj.hist_enc_r(obj.hist_enc_currFrame) - obj.hist_enc_r(obj.hist_enc_currFrame-1))/dt;
                vl = (obj.hist_enc_l(obj.hist_enc_currFrame) - obj.hist_enc_l(obj.hist_enc_currFrame-1))/dt;
             else
                vr = 0;
                vl = 0;
             end
             
             obj.hist_bodyV(obj.hist_currFrame) = (vr+vl)/2;
             obj.hist_bodyOmega(obj.hist_currFrame) = (vr-vl)/(obj.WHEEL_TREAD);
             
             obj.hist_th(obj.hist_currFrame) = obj.hist_th(obj.hist_currFrame-1)+(obj.hist_bodyOmega(obj.hist_currFrame)*dt);
             
             obj.hist_x(obj.hist_currFrame)=obj.hist_x(obj.hist_currFrame-1)+(obj.hist_bodyV(obj.hist_currFrame)*dt)*cos(obj.hist_th(obj.hist_currFrame));
             obj.hist_y(obj.hist_currFrame)=obj.hist_y(obj.hist_currFrame-1)+(obj.hist_bodyV(obj.hist_currFrame)*dt)*sin(obj.hist_th(obj.hist_currFrame));
             
             obj.hist_th(obj.hist_currFrame)
             obj.hist_enc_currFrame = obj.hist_enc_currFrame + 1;
             obj.hist_currFrame = obj.hist_currFrame + 1;
         end 
             
             
         function logEncoders(obj)
            obj.core.encoders.NewMessageFcn = @obj.encoderEventListener;
            
         end
    end % P2_Robot->methods
    
end % P2_Robot