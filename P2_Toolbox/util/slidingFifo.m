classdef slidingFifo < GenericDataContainer
    %slidingFifoQueue A quick and dirty FIFO ques done with short vectors
    %so that you can run interp1 on the result to interpolate in time.
    %Intended to be used for modelling delays in real-time system. The
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
        
        function obj = slidingFifo(maxElements, first_elem)
        % Construct a slidingFifo Queue.
            if  nargin > 0
                obj.maxElements = maxElements;
                
                if nargin > 1
                    obj.que = first_elem;
                else
                    obj.que = [];
                end
            end % nargin>0?
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
    end % slidingFifo <- methods
    
    % Implement Abstract Methods of GenericDataContainer:
    methods
        % Returns the First Element of the Data Set
        function e = first(obj); e = obj.que(1); end
        % Returns the Last Element of the Data Set
        function e = last(obj); e = obj.que(end); end
        
        % Returns the Nth Element of the Data Set (1-indexed)
        function e = nth(obj,n); e = obj.que(n); end
        % Returns the Nth Element from the End of the Data Set (1-indexed)
        function e = nthFromEnd(obj,n); e = obj.que(end-n); end
        
        % Returns a Vector Representing the State of the Data Set at the
        % Current Moment
        function es = vec(obj); es = obj.que; end
    end % slidingFifo <- methods(Abstract)
end