% Abstract SuperClass for All Trajectory Planning Curves, defining a
% standard set of external access methods and properties so that algorithms
% using one RT (ex TTC) can easily substitute it for another (ex. TCS).
classdef (Abstract) ReferenceTrajectory < Trajectory
    properties(GetAccess=public, SetAccess=public)
        init_pose;          % Initial Pose of the Trajectory
        transformed = 0;    % Whether the Entire Trajectory has been Transformed
    end % ReferenceTrajectory <-properties(public,public)
    
    properties(Abstract)
        send_delay;     % s, Delay from Command Send to When the Robot Begins Executing it.
        
        N_samples;      % Number of Samples in the Trajectory
    end % ReferenceTrajectory <-properties(Abstract)
    
    methods(Static)
       
        % Transforms a Pose from Path-Relative Coordinates to World 
        % Coordinates given an initial position, p0
        function pw = poseToWorld(pr, p0)
            
        end
        
        
    end % ReferenceTrajectory <-methods(Static)
    
    methods(Abstract)
        
        % Compute Everything Necessary to Create a Follow-able Path
        compute(obj);
        
        % Transforms Every Pose in the Data-Set to World Coordinates based
        % on the Object's "init_pose" property.
        offsetInitPose(obj);
        
    end % ReferenceTrajectory <-methods(Abstract)
    
end % Class ReferenceTrajectory