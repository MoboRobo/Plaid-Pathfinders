% Abstract SuperClass for All Trajectory Planning Curves, defining a
% standard set of external access methods and properties so that algorithms
% using one RT (ex TTC) can easily substitute it for another (ex. TCS).
classdef (Abstract) ReferenceTrajectory
    properties(Abstract)
        send_delay;     % s, Delay from Command Send to When the Robot Begins Executing it.
        
        N_samples;      % Number of Samples in the Trajectory
        resolution;     % s, Separation between Times
        
        t_init;         % s, Curve Seed Time
        t_fin;          % s, Curve End Time
    end % ReferenceTrajectory <-properties(Abstract)
    
    methods (Abstract)
        
    end % ReferenceTrajectory <-methods(Abstract)
end % Class ReferenceTrajectory