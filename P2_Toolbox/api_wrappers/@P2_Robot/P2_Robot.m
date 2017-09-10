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
            
            obj.on_time = tic;
            obj.trip_startTime = tic;
            obj.init_enc_l = obj.core.encoders.LatestMessage.Vector.X;
            obj.init_enc_r = obj.core.encoders.LatestMessage.Vector.Y;
            obj.startTrip(); % Default odometry starts at instantiation.
        end % #P2_Robot Constructor
        
        %% Destructor
        function delete(obj)
            obj.core.delete(); %Destruct Core Robot
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
        
        %%DEPRECATED%%
%         % Go to Specified One-Dimensional Position relative to current
%         % position with velocity curve starting at ps (peak speed) and
%         % decelerating to MIN_SPEED exponentially after half the distance
%         % has been covered.
%         % (N.B.: resets trip odometer)
%         function goTo_straight(obj, pos, ps)
%             global debugger
%             
%             obj.startTrip()
%             
%             strt = obj.avgTripDist(); %starting position
%             D_TS = pos - strt; %delta btwn starting and target position
%             abs_dts = abs(D_TS);
% 
%             D_TC = pos - strt; %delta btwn Current and Target position
%             abs_dtc = abs(D_TC);
% 
%             spd = ps;
%             while (spd ~= 0)
%                 ad = obj.avgTripDist();
% 
%                 D_CS = ad - strt; %delta btwn Start and Current Position
%                 abs_dcs = abs(D_CS);
% 
%                 D_TC = pos - ad; %delta btwn Current and Target Position
%                 abs_dtc = abs(D_TC);
% 
%                 if(abs_dcs > abs_dts/2) %if past half-way
%                     spd = (ps-obj.MIN_SPEED) * abs_dtc/(abs_dts/2) + obj.MIN_SPEED;
%                 end% |DCS|>|DTS|/2?
%                 if(abs_dcs > abs_dts) %if done.
%                     spd = 0;
%                 end % |DCS|>|DTS|?
% 
%                 if(D_TS < 0) %Go Backwards
%                     if(~isempty(debugger))
%                         debugger.echo([obj.avgTripDist() pos -spd]);
%                     end
%                     obj.run_correct(-spd,-spd);
%                 else %Go Forward
%                     if(~isempty(debugger))
%                         debugger.echo([obj.avgTripDist() pos spd]);
%                     end
%                     obj.run_correct(spd,spd);
%                 end % D_TS<0?
% 
%                 pause(0.05)
%                 if(~isempty(debugger))
%                     debugger.plot_encoderPosition(0.4);
%                 end % ~isempty(debugger)?
%             end %while(spd~=0)
%         end % #goTo_straight
    end % P2_Robot->methods
end % P2_Robot