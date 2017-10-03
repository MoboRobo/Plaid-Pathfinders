classdef ROSCallbackData < event.EventData
    %ROSCallbackData simple callback emulates robot. 
    
    properties
        LatestMessage
    end
    
    methods
        function obj = ROSCallbackData(LatestMessage)
            obj.LatestMessage = LatestMessage;
        end
    end
    
end

