% Abstract SuperClass for All Trajectory Planning Curves, defining a
% standard set of external access methods and properties so that algorithms
% using one RT (ex TTC) can easily substitute it for another (ex. TCS).
% ... sadly (but obviously), this requires Heterogeneous Mixins and sealed 
% data access
classdef (Abstract) ReferenceTrajectory < Trajectory & matlab.mixin.Heterogeneous
    %% PROPERTIES
    properties(GetAccess=public, SetAccess=public)
        init_pose;          % Initial Pose of the Trajectory
        transformed = 0;    % Whether the Entire Trajectory has been Transformed
        
        is_null = 0;        % Red flag to not mess with null methods.
    end % ReferenceTrajectory <-properties(public,public)
    
    properties(GetAccess=public, SetAccess=protected)
        numSamples = 0; % Number of Samples in the Trajectory
        distArray = [];
        timeArray = [];
        curvArray = [];
        vlArray = [];
        vrArray = [];
        VArray = [];
        wArray = []; % "w" = Name Signature used in Provided TCS Framework.
        poseArray = [];
        
        method = 'pchip';  % Interpolation Method
    end % ReferenceTrajectory <-properties(public,private)
    
    properties(Abstract)
        
    end % ReferenceTrajectory <-properties(Abstract)
    
    
    %% METHODS
    methods(Abstract)
        
        % Compute Everything Necessary to Create a Follow-able Path
        compute(obj);
        
    end % ReferenceTrajectory <-methods(Abstract)
    
    % Transformation Functions
    methods(Sealed)
        function transMat = getTransformMat(obj)
            transMat = obj.init_pose.bToA();
        end % #getTransformMat
        
        % Transforms Every Pose in the Data-Set to World Coordinates based
        % on the Object's "init_pose" property.
        function offsetInitPose(obj)
            transformMat = obj.getTransformMat();
            offsetTh = obj.init_pose.th;
            
            % iterate through all x, y, and th in poseArray and transform
            % them
            xs = obj.getXVec();
            ys = obj.getYVec();
            ths = obj.getThVec();
            for i=1:obj.numSamples
                oldTh = ths(i);
                oldX = xs(i);
                oldY = ys(i);
                oldPose = [oldX; oldY; 1];
                
                newPose = transformMat * oldPose;
                xs(i) = newPose(1);
                ys(i) = newPose(2);
                ths(i) = atan2(sin(oldTh + offsetTh), cos(oldTh + offsetTh));
            end
            obj.poseArray(1,:) = xs;
            obj.poseArray(2,:) = ys;
            obj.poseArray(3,:) = ths;
            obj.transformed = 1; %inform callback that the commands have
                                 %been transformed
        end % #offsetInitPose
    end % ReferenceTrajectory <-methods(Sealed)
    
    % Sealed Accessors
    methods(Sealed)
        %% Interpolate
        % General Interpolation that takes into account out of bounds
        % queries, where v_ob is the value returned for over-bounds,
        % queries and v_ub is the value returned for under-bounds queries.
        function v = getInt(obj,xs,vs, x, v_ob, v_ub)
            if ~obj.is_null
                if obj.numSamples > 0

                    if (x < xs(1))
                        v = v_ub;
                    elseif(x <= xs(obj.numSamples))
                        v = interp1(xs,vs, x, obj.method);
                    else
                        v = v_ob;
                    end % x?

                else
                    v = v_ub;
                end
            else
                v = 0; % Nulls spit out all 0's.
            end
        end % #getInt
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - CURVE (SPATIAL)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             
        % Time at Path Length:
        function t = getTAtDist(obj,s)
            t = obj.getInt( ...
                    obj.distArray,obj.timeArray, s, ...
                    obj.getFinalTime(), 0.0 ...
                );
        end % #getTimeAtDist
        
        function s  = getDistAtDist(obj,s)
            s = obj.getInt( ...
                    obj.distArray,obj.distArray, s, ...
                    obj.distArray(obj.numSamples), 0.0 ...
                );
        end % #getDistAtDist
        
        
        % Velocity at Path Length:
        function V = getVAtDist(obj,s)
            V = obj.getInt( ...
                    obj.distArray,obj.VArray, s, ...
                    obj.VArray(obj.numSamples), 0.0 ...
                );
        end % #getVAtDist
        % Angular Velocity at Path Length:
        
        function om = getOmegaAtDist(obj,s)
            om = obj.getInt( ...
                    obj.distArray,obj.wArray, s, ...
                    obj.wArray(obj.numSamples), 0.0 ...
                );
        end % #getOmegaAtDist
        function w  = getwAtDist(obj,s) % Alias from TCS Framework
            w = getOmegaAtDist(obj,s);
        end % #getwAtDist
        
        function K  = getCurvAtDist(obj,s)
            K = obj.getInt( ...
                    obj.distArray,obj.curvArray, s, ...
                    obj.curvArray(obj.numSamples), 0.0 ...
                );
        end % #getCurvAtDist
        
            
        function p  = getPoseAtDist(obj,s)
            xs = obj.xs(); % Vector of X-Positions
            ys = obj.ys();
            ths = obj.ths();
            
            x = obj.getInt(obj.distArray,xs, s, xs(obj.numSamples),obj.init_pose.X);
            y = obj.getInt(obj.distArray,ys, s, ys(obj.numSamples),obj.init_pose.Y);
            th = obj.getInt(obj.distArray,ths, s, ths(obj.numSamples),obj.init_pose.th);
            
            p = pose(x,y,th);
            
            if ~obj.transformed
                %then transform this badboy before we're finished
                p = obj.poseToWorld(p,obj.init_pose);
            end % transformed?
        end % #getPoseAtDist
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - SIGNAL (TRANSIENT)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
              
        %For Inverting Parameterization if Necessary (Computationally
        %heavy)
        % Path Length at t:
        function s = getSAtTime(obj,t)
            s = obj.getInt( ...
                    obj.timeArray,obj.distArray, t, ...
                    obj.getFinalDist(), 0.0 ...
                );
        end % #getSAtTime
        
        
        function V  = getVAtTime(obj,t)
            V = obj.getInt( ...
                    obj.timeArray,obj.VArray, t, ...
                    obj.VArray(obj.numSamples), 0.0 ...
                );
        end % #getVAtTime
           
        % Angular Velocity at Time:
        function om = getOmegaAtTime(obj,t)
            om = obj.getInt( ...
                    obj.timeArray,obj.wArray, t, ...
                    obj.wArray(obj.numSamples), 0.0 ...
                );
        end % #getOmegaAtTime
        function w  = getwAtTime(obj,t) % Required Alias for TCS Framework
            w = getOmegaAtTime(obj,t);
        end % #getwAtTime
        
        % Path Curvature at Time:
        function K = getCurvAtTime(obj,t)
            K = obj.getInt( ...
                    obj.timeArray,obj.curvArray, t, ...
                    obj.curvArray(obj.numSamples), 0.0 ...
                );
            % Why was this here?:
%             K = obj.getCurvAtDist(obj.getSAtTime(t));
        end % #getCurvAtTime
            
        
        function p  = getPoseAtTime(obj,t)
            xs = obj.xs(); % Vector of X-Positions
            ys = obj.ys();
            ths = obj.ths();
            
            x = obj.getInt(obj.timeArray,xs, t, xs(obj.numSamples),obj.init_pose.X);
            y = obj.getInt(obj.timeArray,ys, t, ys(obj.numSamples),obj.init_pose.Y);
            th = obj.getInt(obj.timeArray,ths, t, ths(obj.numSamples),obj.init_pose.th);
            
            p = pose(x,y,th);
            
            if ~obj.transformed
                %then transform this badboy before we're finished
                p = obj.poseToWorld(p,obj.init_pose);
            end % transformed?
        end % #getPoseAtTime
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - TERMINAL (END STATE)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function p  = getFinalPose(obj)
            if ~obj.is_null
                if obj.numSamples > 0
                    p = pose(obj.poseArray(:,obj.numSamples));
                    
                    if ~obj.transformed
                        %then transform this badboy before we're finished
                        p = obj.poseToWorld(p,obj.init_pose);
                    end % transformed?
                else
                    try
                        p = obj.init_pose;
                    catch
                        p = pose(0,0,0); % Return 0s if Init-Pose Not Set
                    end
                end
            else
                p = pose(0,0,0);
            end
        end % #getFinalPose
        
        function time  = getTrajectoryDuration(obj)
            if ~obj.is_null
                if obj.numSamples > 0
                    time  = obj.timeArray(obj.numSamples);
                else
                    time = 0;
                end
            else
                time = 0;
            end
        end % #getTrajectoryDuration
        %Return the Elapsed Time at the End of the Path:
        function tf = getFinalTime(obj)
            tf = obj.getTrajectoryDuration();
        end
        
        function dist  = getTrajectoryDistance(obj)
            if ~obj.is_null
                if obj.numSamples > 0
                    dist  = obj.distArray(obj.numSamples);
                else
                    dist = 0;
                end
            else
                dist = 0;
            end
        end % #getTrajectoryDistance
        %Return the Path Length Covered at the End of the Path:
        function sf = getFinalDist(obj)
            sf = obj.getTrajectoryDistance();
        end
        
        %Returns the Velocity at the End of the Path:
        function vf = getFinalVelocity(obj); vf = obj.V_t(obj.t_f); end
        %Returns the Velocity at the End of the Path:
        function omf = getFinalOmega(obj); omf = obj.om_t(obj.t_f); end
        %Returns the Velocity at the End of the Path:
        function Kf = getFinalCurv(obj); Kf = obj.K_t(obj.t_f); end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - VECTORS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %Returns Vector of All Times:
        function ts = getTVec(obj)
            ts = obj.timeArray(1:obj.numSamples);
        end
        %Returns Vector of All Path Lengths:
        function ss = getSVec(obj)
            ss = obj.distArray(1:obj.numSamples);
        end
        
        
        %Returns Vector of All Velocities:
        function Vs = getVVec(obj)
            Vs = obj.VArray(1:obj.numSamples);
        end
        %Returns Vector of All Rotational Velocities:
        function oms = getOmVec(obj)
            oms = obj.wArray(1:obj.numSamples);
        end
        %Returns Vector of All Curvatures:
        function Ks = getKVec(obj)
            Ks = obj.curvArray(1:obj.numSamples);
        end
        
        
        %Returns Vector of All Poses:
        function ps = getPVec(obj)
            ps = obj.poseArray(1:obj.numSamples);
        end
        
        %Returns Vector of All X-Positions:
        function xs = getXVec(obj)
            xs = obj.poseArray(1,1:obj.numSamples);
        end
        %Returns Vector of All Y-Positions:
        function ys = getYVec(obj)
            ys = obj.poseArray(2,1:obj.numSamples);
        end
        %Returns Vector of All Headings:
        function ths = getThVec(obj)
            ths = obj.poseArray(3,1:obj.numSamples);
        end
        
    end % ReferenceTrajectory <-methods(Sealed)
    
end % Class ReferenceTrajectory