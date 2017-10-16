classdef TestContainer < handle
    properties(Constant)
        max_queue_size = 10000; % Max Size of Each Data Set
    end
    properties(GetAccess=public, SetAccess=private)
        data_A;
    end
    
    methods
        function obj = TestContainer()
            obj.data_A = slidingFifo(TestContainer.max_queue_size, pose(0,0,0));
        end
        
        function update(obj, val)
            x = length(obj.data_A.vec());
            y = length(obj.data_A.vec()) / 2.0;
            th = val / 2.0;
            obj.data_A.add( pose(x,y,th) );
        end
    end
end