% Defines attributes of a Job for MRPL and necessary Procedures to Execute
% it. Pass the Constructor the job id and the arguments for the function.
classdef Job
    %% PROPERTIES
    properties(Access=private)
        job_id; % ID of Job
        
        % Container Struct for Properties of Each Job
        props = struct('duration', 0);
        
        % Cell Array of Arguments given to this Job.
        args;
        
        % Anonymous Function Handle for Function(s) to Execute when
        % Performing this Job (must be given a mrplSystem, mrpl).
        exec_fcn;
        
    end % Job <- properties(private)
    
    %% METHODS
    methods
        %% Constructor
        function obj = Job(id, varargin)
            obj.job_id = id;
            
            obj.args = varargin;
            
            switch id
                case {JID.PICK, JID.DROP}i89io
                    obj.props.pose = varargin{1};
                    obj.props.speed = varargin{2};
                    if id == JID.PICK
                        obj.exec_fcn = @(mrpl) mrpl.pickupObjAt(obj.args);
                    else
                        obj.exec_fcn = @(mrpl) mrpl.dropObjAt(obj.args);
                    end % id?
                    
                case JID.WAIT
                    obj.props.duration = varargin{1};
                    obj.exec_fcn = @(~) pause(obj.args);
                    
            end % switch id
            
        end % #Job
        
    end % Job <- methods
end % class Job