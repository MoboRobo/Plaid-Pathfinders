classdef slidingFifo < GenericDataContainer
    % Pre-Allocated Sliding Fifo Queue.
    % Deprecated Description (by Al Kelly):
    %slidingFifoQueue A quick and dirty FIFO ques done with short vectors
    %so that you can run interp1 on the result to interpolate in time.
    %Intended to be used for modelling delays in real-time system. The
    %length of the queue should be small for performance reasons.
    
    % interp1 requires that the "x" array be monotone and have unique
    % values in it. That means this que has to grow to some length by
    % adding at the right and then start throwing data out the left.
    
    properties(Constant)
    end
    
    properties(GetAccess=public, SetAccess=private)
        maxElements;
    end
    
    properties(Access=public)
        que;
        numElements; % Number of Elements Currently Populated
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
                
                % N.B. Ensure Vector Homogen. in que.
                if nargin > 1
                    obj.que = first_elem;
                    obj.que(2:maxElements) = first_elem; % Ensure Class Consistency
                    % ^ TODO: Find way to get class of first_elem and use
                    % empty of that class:
                    obj.numElements = 1;
                else
                    obj.que(1:maxElements) = 0.0; % double by default.
                    obj.numElements = 0;
                end
            else
                error('Class slidingFifo Constructor must be supplied at least one argument');
            end % nargin>0?
        end 
        
        % Adds the given element to the end of the queue.
        function add(obj,element)
        % Add an object at the right of the queue and slide to the left.
            if(obj.numElements == obj.maxElements)
                obj.que(1:end-1) = obj.que(2:end);
                obj.que(end) = element;
            else
                obj.numElements = obj.numElements + 1;
                obj.que(obj.numElements) = element;
            end
        end % #add
        % Alias for #add.
        function push(obj,element); obj.add(element); end % #push
        
        % Adds the Given Element to the Queue by Making it the nth-Element
        function addAt(obj, element, n)
            if(n > obj.numElements || n<1)
                warning(strcat('Nth element ', n, 'is not populated in call to #slidingFifo::addAt'));
            elseif(obj.numElements+1 > obj.maxElements)
                warning(strcat('FiFo Addition Failed at #slidingFifo::addAt.', ...
                               ' Inserting an element into this queue will cause it to overflow and produce undefined behaviour.'));        
%                 % If adding an element is going to throw the list
%                 % overbounds, kick out the first element. Do this /first/
%                 % to avoid interim data reallocation for a larger array.
%                 if(obj.numElements+1 > obj.maxElements)
%                     obj.pop(); %Takes care of resizing numElements
%                     idx = idx - 1; % Index is now smaller
%                 end
            else
                idx = n;
                obj.que(idx+1:obj.numElements+1) = obj.que(idx:obj.numElements);
                obj.que(idx) = element;
                obj.numElements = obj.numElements + 1;
            end
        end % #addAt
        
        % Removes the First Element from the Queue and Returns It.
        function e = pop(obj)
            if(obj.numElements > 0)
                e = obj.que(1);
                obj.que(1:obj.numElements-1) = obj.que(2:obj.numElements);
                obj.que(obj.numElements) = obj.que(end);% Ensure Matching of Empty Data Type Class
                obj.numElements = obj.numElements - 1;
            else
                e = obj.que(end); % Ensure Matching of Empty Data Type Class
                warning(strcat('No elements populated in call to #slidingFifo::pop'));
            end %numElements?
        end % #pop
        
        % Alias for First. Returns the First Element of the Queue.
        function e = peek(obj); e = obj.first(); end % #peek
        
        % Removes the nth element from the queue and returns it.
        function e = remove(obj, n)
            if(n > obj.numElements || n<1)
                warning(strcat('Nth element ', n, 'is not populated in call to #slidingFifo::remove'));
            else
                e = obj.que(n);
                obj.que(n:obj.numElements-1) = obj.que(n+1:obj.numElements);
                obj.que(obj.numElements) = obj.que(end);% Ensure Matching of Empty Data Type Class
                obj.numElements = obj.numElements - 1;
            end
        end % #remove
        
        % Reverses the Queue by Reversing the Populated Section of the 
        % Allocated Data Space.
        function reverse(obj)
            % Just did this because MATLAB has a cool way of doing it.
            if(obj.numElements > 0)
                obj.que(1:obj.numElements) = obj.que(obj.numElements:-1:1);
            else
                warning(strcat('Queue has zero length in call to #slidingFifo::reverse'));
            end
        end % #reverse
        
    end % slidingFifo <- methods
    
    % Implement Abstract Methods of GenericDataContainer:
    methods
        % Returns the First Element of the Data Set
        function e = first(obj)
            if(obj.numElements > 0)
                e = obj.que(1);
            else
                e = obj.que(end); % Ensure Matching of Empty Data Type Class
                warning(strcat('No elements populated in call to #slidingFifo::first'));
            end
        end
        % Returns the Last Element of the Data Set
        function e = last(obj)
            if(obj.numElements > 0)
                e = obj.que(obj.numElements);
            else
                e = obj.que(end); % Ensure Matching of Empty Data Type Class
                warning(strcat('No elements populated in call to #slidingFifo::last'));
            end
        end
        
        % Returns the Nth Element of the Data Set (1-indexed)
        function e = nth(obj,n)
            if(n > obj.numElements)
                e = obj.que(end); % Ensure Matching of Empty Data Type Class
                warning(strcat('Nth element ', n, 'is not populated in call to #slidingFifo::nth'));
            else
                e = obj.que(n);
            end
        end
        % Returns the Nth Element from the End of the Data Set (1-indexed)
        function e = nthFromEnd(obj,n)
            if(obj.numElements > 0)
                e = obj.que(obj.numElements-n);
            else
                e = obj.que(end); % Ensure Matching of Empty Data Type Class
                warning(strcat('Nth element ', n, 'is not populated in call to #slidingFifo::nthFromEnd'));
            end
        end
        
        % Returns a Vector Representing the State of the Data Set at the
        % Current Moment
        function es = vec(obj)
            if obj.numElements > 0
                es = obj.que(1:obj.numElements);
            else
                es = obj.que(end); % Ensure Matching of Empty Data Type Class
                warning(strcat('No elements populated in call to #slidingFifo::vec'));
            end
        end
    end % slidingFifo <- methods(Abstract)
end