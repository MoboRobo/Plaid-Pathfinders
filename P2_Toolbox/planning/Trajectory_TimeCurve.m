% Class for Pre-Computing the Trajectory Curve (parameterized by time) of 
% an Arbitrary Path defined by the Function (Handles): V_func(obj,t),
% om_func(obj,t).
% (TODO: Incorporate path reduction algorithm like RDP?)
classdef Trajectory_TimeCurve < ReferenceTrajectory
    %% PROPERTIES
    properties(GetAccess = public, SetAccess = public)
        resolution;         % s, Separation between Times
        
        method = 'spline';  % Interpolation Method (override ReferenceTrajectory default)
        
        t_init = 0;         % s, Curve Seed Time
        t_fin = Inf;        % s, Curve End Time
    end % TrajectoryCurve <- properties(public,public)
    
    properties(GetAccess = public, SetAccess = private)
        V_func; % Anonymous Function Detailing Calculation of V wrt time.
        om_func; % Anonymous Function Detailing Calculation of Omega wrt time.
    end % TrajectoryCurve <- properties(public,private)
    
    %% METHODS
    methods
        %% Constructor
        % Instantiates an instance of the trajectory curve defined by the
        % time-variant functions vf(obj,t), omf(obj,t) across ns samples
        % from time t0 to tf, optionally starting at init_pose
        function obj = Trajectory_TimeCurve(vf, omf, t0,tf, ns, init_pose)
            % Check if all Abstract Methods are Implemented:
            meta.abstractDetails(?Trajectory_TimeCurve)
            
            obj.V_func = vf;
            obj.om_func = omf;
            
            if nargin > 2
                obj.t_init = t0;
            end % nargin>2?
            if nargin > 3
                obj.t_fin = tf;
            end % nargin>3?
            
            if nargin > 4
                obj.numSamples = ns;
            end % nargin>4?
            obj.resolution = (tf-t0)/(obj.numSamples-1);%Beware the fence-post
            
            if nargin > 5
                obj.poses(1) = init_pose;
                obj.init_pose = init_pose;
            else
                obj.init_pose = pose(0,0,0);
            end% nargin>5?
            
            obj.t_init = t0;
            obj.t_fin = tf;
            obj.compute();
        end % Constructor
        
        %% Compute
        % Computes the trajectory from time t0 to tf.
        function compute(obj)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -FINISH THIS
        
            obj.timeArray = (obj.t_init : obj.resolution : obj.t_fin);
            obj.numSamples = length(obj.timeArray); % Fix any anomalous shifting errors
            
            % Sets Velocity Profile (V,om,K) at Index, i, to Velocity V, 
            % and Rotational Velocity, om, while ensuring that P2_Robot's
            % maximum velocity is not exceeded for either the left or right
            % wheel.
            % TODO: Also add accel limiting?
            function setVProfile(idx,V,om)
                [V_set, om_set, vl, vr] = P2_Robot.limitWheelVelocity(V,om);
                
                obj.vlArray(idx) = vl;
                obj.vrArray(idx) = vr;
                
                obj.VArray(idx) = V_set;
                obj.wArray(idx) = om_set;
                obj.curvArray(idx) = om_set / V_set;
            end
            
            %Compute First Point:
            setVProfile(1, obj.V_func(obj,obj.t_init), obj.om_func(obj,obj.t_init));
            
            i = 2; % Account for Starting at init_pose
            while(i <= obj.numSamples)
                t = obj.timeArray(i);
                dt = t - obj.timeArray(i-1);
                
                setVProfile(i, obj.V_func(obj,t), obj.om_func(obj,t));
                
                V = obj.VArray(i-1);
                omega = obj.wArray(i-1);
                
                new_th = obj.poseArray(3,i-1) + omega*dt/2;
                    new_x = obj.poseArray(1,i-1) + V*cos(new_th)*dt;
                    new_y = obj.poseArray(2,i-1) + V*sin(new_th)*dt;
                new_th = new_th + omega*dt/2;
                obj.distArray(i) = obj.distArray(i-1) + V*dt;
                
                obj.poseArray(:,i) = [new_x;new_y;new_th];
            i = i+1;
            end
        end % #compute
        
        
        %% LEGACY ACCESSORS:
        %% Get Pose
        % Returns the pose of the robot at trajectory time, t.
        function p = getPose(obj, t_in)
            p = obj.getPoseAtTime(t_in);
        end % #getPose
        
        %% Get Velocity
        % Returns the velocity of the robot at trajectory time, t.
        function V = getV(obj, t_in)
            V = obj.getVAtTime(t_in);
        end % #getV
        function V = V(obj,t); V=getV(obj,t); end %Shorthand
        
        %% Get Omega
        % Returns the angular velocity of the robot at trajectory time, t.
        function om = getOmega(obj, t_in)
            om = obj.getOmegaAtTime(t_in);
        end % #getPose
        function om = om(obj,t); om=getOmega(obj,t); end %Shorthand

    end % TrajectoryCurve <- methods
    
end % 