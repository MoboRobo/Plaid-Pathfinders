classdef SimSubscriber < handle
	events
        OnMessageReceived % Nofified when message is received by this subscriber
    end
    
    properties
        LatestMessage
        NewMessageFcn
	end
    
    methods
		function obj = SimSubscriber(LatestMessage)
			obj.LatestMessage = LatestMessage;
		end
        function publish(obj)
			notify(obj, 'OnMessageReceived',ROSCallbackData(obj.LatestMessage));
            % Oct 30,2016, Al Changed 2nd argument from ROSCallbackData(obj.LatestMessage)
            if ~isempty(obj.NewMessageFcn)
                obj.NewMessageFcn(obj,obj.LatestMessage); 
            end
        end
    end 
end