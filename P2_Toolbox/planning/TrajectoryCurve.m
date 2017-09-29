% Class for Pre-Computing the Trajectory Curve of an Arbitrary Path defined
% by the Anonymous Functions of time, V_func, om_func.
% (TODO: Incorporate path reduction algorithm like RDP?)
classdef Trajectory_TimeCurve < handle 
    %% PROPERTIES
    properties(GetAccess = public, SetAccess = public)
        send_delay = 0.1;   % s, Delay from Command Send to When the Robot Begins Executing it.
        resolution = 0.1;   % s, resolution of calculations (step size)
    end % TrajectoryCurve <- properties(public,public)
    
    properties(GetAccess = public, SetAccess = private)
        velProfile = [struct('V',0, 'om',0)];
        
        V_func; % Anonymous Function Detailing Calculation of V wrt time.
        om_func; % Anonymous Function Detailing Calculation of Omega wrt time.
    end % TrajectoryCurve <- properties(public,private)
    
    %% METHODS
    methods
        %% Constructor
        function obj = TrajectoryCurve(vf, omf)
            
        end % Constructor
        
        %% Compute
        % Computes the trajectory.
        function compute()
            
        end
            
        get_X(obj, t)
        
        get_s(obj, t)
        
        get_v(obj, t)
        
        get_omega(obj, t)
    end % TrajectoryCurve <- methods
    
end % 