classdef ProbRob < handle
    properties(GetAccess=public, SetAccess=private)
        test_prop;
        
        on_time;
    end
    
    methods
        function obj = ProbRob
            obj.test_prop = TestContainer();
            obj.on_time = tic;
        end
        
        function updateTheThing_Callbackesque(obj, ~, ~)
            obj.test_prop.update( toc(obj.on_time) );
        end
    end
end