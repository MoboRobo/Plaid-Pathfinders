% Wrapper for a Generic Clock (CPU by default) which allows for stopping
% the passage of time (pausing) for debugging purposes.
classdef Clock < handle
    %% PROPERTIES
    properties(GetAccess = public, SetAccess = public)
        start_time;     % s, Initial Time Signature (depends on time method,
                        % usually an @tic result;
                        
        get_time;       % @, Anonymous function for getting elapsed time
        init_time;      % @, Anonymous function for setting initial time
        
        paused = 0;     % bool, Whether the clock is currently paused
        prior_time = 0; % s, Amount of time elapsed before the clock resumed
    end % Clock <- properties(public,public)
    
    %% METHODS
    methods
        %% Constructor
        % Initialize a new clock instance by calling Clock(). Optionally,
        % function handles can be given to set the initial time ( it() )and
        % get the current time ( gt() ).
        function obj = Clock(it, gt)
            obj.init_time = @tic;
            if nargin > 0
                obj.init_time = it;
            end % nargin>0?
            
            if nargin > 1
                obj.get_time = gt;
            else
                obj.get_time = @()toc(obj.start_time);
            end % nargin>1?
            
            obj.start_time = obj.init_time();
        end % Clock Constructor
        
        %% Time
        % Returns the current time.
        function t = time(obj)
            if(~obj.paused)
                t = obj.prior_time + obj.get_time();
            else
                t = obj.prior_time;
            end
        end
        
        %% Pause
        % Pauses the passage of time for this clock; thus, if for but an
        % instant, suspending the mortal coil of our belove'd robot,
        % halting its unrelenting march towards its inevitable demise.
        % ... and returns the current time at the time of the pause.
        function t = pause(obj)
            t = obj.time(); % Gather pause time immediately.
            obj.prior_time = t;
            obj.paused = 1;
        end
        
        %% Resume
        % Resumes the passage of time for this clock.
        function resume(obj)
            obj.start_time = obj.init_time();
            obj.paused = 0;
        end
            
    end % Clock <- methods
end % Clock