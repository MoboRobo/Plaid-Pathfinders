classdef JobScheduler < handle
    properties
        jobs = slidingFifo.empty;
        mrpl;
    end % JobScheduler <- properties
    
    methods
        %% Constructor
        function obj = JobScheduler(m)
            obj.mrpl = m;
        end
        
        %% Schedule
        %takes in an array of jobs and stores them in the scheduler!
        function schedule(obj, jobsToBeScheduled)
            first = jobsToBeScheduled(1);
            obj.jobs = slidingFifo(1000, first);
            for job = jobsToBeScheduled(2:end)
                obj.jobs.add(job);
            end
        end
       
        %% Execute Next
        % Performs the next job in scheduler.
        function executeNext(obj)
            nextJob = obj.jobs.pop();
            nextJob.exec_fcn(obj.mrpl);
        end % end
        
%         %% Execute
%         % Performs the given Job.
%         function(obj, job)
%             
%         end % end
    end % JobScheduler <- methods
    
    
end % class JobScheduler