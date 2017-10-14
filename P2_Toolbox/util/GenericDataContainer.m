% Abstract Set of Generic Properties/Methods Every Data Container Should
% Have
classdef (Abstract) GenericDataContainer < handle
    %% PROPERTIES
    properties(Abstract)
        
    end % GenericDataContainer <- properties(Abstract)
    
    %% METHODS
    methods(Abstract)
        % Returns the First Element of the Data Set
        e = first(obj);
        % Returns the Last Element of the Data Set
        e = last(obj);
        
        % Returns the Nth Element of the Data Set (1-indexed)
        e = nth(obj,n);
        % Returns the Nth Element from the End of the Data Set (1-indexed)
        e = nthFromEnd(obj,n);
        
        % Returns a Vector Representing the State of the Data Set at the
        % Current Moment
        es = vec(obj);
    end % GenericDataContainer <- methods(Abstract)
end