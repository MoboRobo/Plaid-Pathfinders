% Class for Pre-Computing the Trajectory Curve (parameterized by time) of 
% an Arbitrary Path defined by the Function (Handles): V_func(obj,t),
% om_func(obj,t).
% (TODO: Incorporate path reduction algorithm like RDP?)
classdef Trajectory_TimeCurve < handle
    %% PROPERTIES
    properties(GetAccess = public, SetAccess = public)
        send_delay = 0.352;   % s, Delay from Command Send to When the Robot Begins Executing it.
        
        N_samples = 100;    % Number of Samples in the Trajectory
        resolution;         % s, Separation between Times
        
        method = 'spline';  % Interpolation Method
        
        t_init = 0;         % s, Curve Seed Time
        t_fin = Inf;        % s, Curve End Time
    end % TrajectoryCurve <- properties(public,public)
    
    properties(GetAccess = public, SetAccess = private)
        times = [0];                        % Vector of Times for each 
                                            % index
        profile = [struct('V',0, 'om',0)];  % Vector of all V's and om's
                                            % defining the trajectory
        poses = [pose(0,0,0)];              % Vector of all Poses
        
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
            obj.V_func = vf;
            obj.om_func = omf;
            
            if nargin > 2
                obj.t_init = t0;
            end % nargin>2?
            if nargin > 3
                obj.t_fin = tf;
            end % nargin>3?
            
            if nargin > 4
                obj.N_samples = ns;
            end % nargin>4?
            obj.resolution = (tf-t0)/(obj.N_samples-1);%Beware the fence-post
            
            if nargin > 5
                obj.poses(0) = init_pose;
            end % nargin>5?
            
            obj.compute(t0,tf);
%             transpose([obj.poses(:).poseVec])
        end % Constructor
        
        %% Compute
        % Computes the trajectory from time t0 to tf.
        function compute(obj,t0,tf)
            obj.times = (t0 : obj.resolution : tf);
            
            %Compute First Point:
            obj.profile(1) = struct( ...
                'V', obj.V_func(obj,t0), ...
                'om', obj.om_func(obj,t0) ...
            );
        
            i = 2;
            while(i <= obj.N_samples)
                t = obj.times(i);
                dt = t - obj.times(i-1);
                
                obj.profile(i) = struct( ...
                    'V', obj.V_func(obj,t), ...
                    'om', obj.om_func(obj,t) ...
                );
                
                V = obj.profile(i-1).V;
                omega = obj.profile(i-1).om;
                
                new_th = obj.poses(i-1).th + omega*dt/2;
                new_x = obj.poses(i-1).x + V*cos(new_th)*dt;
                new_y = obj.poses(i-1).y + V*sin(new_th)*dt;
                new_th = new_th + omega*dt/2;
                
                obj.poses(i) = pose(new_x,new_y,new_th);
            i = i+1;
            end
        end % #compute
            
        %% Interpolate
        % General Interpolation that takes into account out of bounds
        % queries, where v_ob is the value used for over-bounds queries.
        function v = genInt(obj,ts,vs, t, v_ob)
            if(t < obj.t_fin)
                v = interp1(ts,vs, t, obj.method);
            else
                v = v_ob;
            end % t<t_fin?
        end
        
        %% Get Pose
        % Returns the pose of the robot at trajectory time, t.
        function p = getPose(obj, t_in)
            t = t_in - obj.send_delay;
            xs = obj.xs(); % Vector of X-Positions
            ys = obj.ys();
            ths = obj.ths();
            
            x = obj.genInt(obj.times,xs, t, xs(end));
            y = obj.genInt(obj.times,ys, t, ys(end));
            th = obj.genInt(obj.times,ths, t, ths(end));
            
            p = pose(x,y,th);
        end % #getPose
        
        %% Get Velocity
        % Returns the velocity of the robot at trajectory time, t.
        function V = getV(obj, t_in)
            t = t_in - obj.send_delay;
            vs = [obj.profile(:).V];
            V = obj.genInt(obj.times,vs, t, 0);
        end % #getV
        function V = V(obj,t); V=getV(obj,t); end %Shorthand
        
        %% Get Omega
        % Returns the angular velocity of the robot at trajectory time, t.
        function om = getOmega(obj, t_in)
            t = t_in - obj.send_delay;
            oms = [obj.profile(:).om];
            om = obj.genInt(obj.times,oms, t, 0);
        end % #getPose
        function om = om(obj,t); om=getOmega(obj,t); end %Shorthand
        
        %% Get X Vec
        % Returns a Vector of X-Positions
        function xs = getXVec(obj)
            xs = [obj.poses(:).poseVec]; xs = xs(1,:);
        end % #getXVec
        function xx = xs(obj); xx=getXVec(obj); end %Shorthand
        %% Get Y Vec
        % Returns a Vector of Y-Positions
        function ys = getYVec(obj)
            ys = [obj.poses(:).poseVec]; ys = ys(2,:);
        end % #getYVec
        function yy = ys(obj); yy=getYVec(obj); end %Shorthand
        %% Get Th Vec
        % Returns a Vector of X-Positions
        function ths = getThVec(obj)
            ths = [obj.poses(:).poseVec]; ths = ths(3,:);
        end % #getThVec
        function tths = ths(obj); tths=getThVec(obj); end %Shorthand
        

    end % TrajectoryCurve <- methods
    
end % 