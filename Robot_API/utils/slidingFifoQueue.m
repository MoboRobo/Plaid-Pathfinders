classdef slidingFifoQueue < handle
    %slidingFifoQueue A quick and dirty FIFO queue done with short vectors
    %so that you can run interp1 on the result to interpolate in time.
    %Intended to be used for modelling delays in real-time systems. The
    %length of the queue should be small for performance reasons.
    
    % interp1 requires that the "x" array be monotone and have unique
    % values in it. That means this que has to grow to some length by
    % adding at the right and then start throwing data out the left.
    
    properties(Constant)
    end
    
    properties(Access = private)
        maxElements;
    end
    
    properties(Access = public)
        que;
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = private)
         
    end
            
    methods(Access = public)
        
        function obj = slidingFifoQueue(maxElements)
        % Construct a slidingFifoQueue.
            if  nargin > 0
                obj.maxElements = maxElements;
                obj.que = [];
            end
        end
        
        function add(obj,element)
        % Add an object at the right of the queue and slide to the left.
            if(length(obj.que) == obj.maxElements)
                obj.que(1:end-1) = obj.que(2:end);
                obj.que(end) = element;
            else
                obj.que(end+1) = element;
            end
            
        end       
    end
end