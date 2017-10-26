%% Null Child of the Trajectory Cubic Spiral
classdef Null_TCS < Trajectory_CubicSpiral
    methods
        function obj = Null_TCS()
            obj = obj@Trajectory_CubicSpiral([0 0 0], 3);
            obj.is_null = 1;
        end
    end
end