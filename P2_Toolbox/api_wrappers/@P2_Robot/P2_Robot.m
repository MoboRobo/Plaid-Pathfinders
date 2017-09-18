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
        on_time;        % Time (tic) that the robot started
        
        %Motion:
        curr_V = 0;     % m/s, Most Recently Commanded Body Velocity
        curr_omega = 0; % rad/s, Most Recently Commanded Rotational Velocity
        
        %Raspbot Data Logs:
        
        % History of Encoder Readings [m]
        % Formatted as: [[sl_0,sr_0, t_0]; ... ]:
        hist_enc = [struct('s_l',0, 's_r',0, 't',0)];
        % History of Robot Positions determined from Sensor Readings.
        % Formatted as: [[x_0,y_0,th_0]; ... ]
        hist_estPose = [struct('X',0, 'Y',0, 'th',0)];
        
        % History of Velocity Commands Issued to the Robot:
        % Formatted as: [[vl_0,vr_0,t_0]; ... ]
        hist_commVel = [struct('v_l',0, 'v_r',0, 't',0)];
        % History of Robot Positions Determined from Input Commands.
        % Formatted as: [[x_0,y_0,th_0]; ... ]
        hist_commPose = [struct('X',0, 'Y',0, 'th',0)];
        
        hist_laser = [[0]]; % m, History of LIDAR Range Reading Vectors
        
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
            obj.core.encoders.NewMessageFcn = @obj.processNewEncoderData;
            
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
        
        %% ODOMETRY
        % Basic position tracking (time and encoder deltas, etc.)
        
        % Processes New Encoder Data by Storing it and Computing Velocity
        % and Dead-Reckiong a Body Position.
        function processNewEncoderData(obj, handle, event)
            %Immediately Capture Event Data:
            s_l = event.Vector.X;
            s_r = event.Vector.Y;
            t = event.Header.Stamp.Sec + event.Header.Stamp.Nsec/1e9;
            
            %Compute Amount of Time Elapsed since Previous Measurement
            dt = t - obj.hist_enc(end).t;

            %Compute Translational Velocity of the Robot's since the last
            %Measurement:
            v_l = (s_l - obj.hist_enc(end).s_l) / dt;
            v_r = (s_r - obj.hist_enc(end).s_r) / dt;
            
            %Now that Previous Command is Done (a new command has been issued),
            %compute IK and robot pose based on how long it was active for.
            [V, omega] = obj.computeIK(v_l, v_r);

            %Update Commanded Odometry (Mid-Point Algorithm):
            new_th = obj.hist_estPose(end).th + omega*dt/2;
            new_x = obj.hist_estPose(end).X + V*cos(new_th);
            new_y = obj.hist_estPose(end).Y + V*sin(new_th);
            new_th = new_th + omega*dt/2;

            obj.hist_enc(end+1) = struct('s_l',s_l, 's_r',s_r, 't',t);
            obj.hist_estPose(end+1) = struct('X',new_x, 'Y',new_y, 'th',new_th);
            
        end % #processNewEncoderData
        
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
        
        % Move the Robot with Left and Right Wheel Speeds v_l and v_r,
        % Logs the Commands into the Command History hist_commVel, and
        % Computes and Logs the Commanded Pose into hist_commPose.
        % (wrapper for the RaspBot setVelocity Command.
        sendVelocity(obj, v_l, v_r);
        
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
        
    end % P2_Robot->methods
    
end % P2_Robot