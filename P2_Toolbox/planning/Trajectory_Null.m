classdef Trajectory_Null < ReferenceTrajectory
    properties(Access = public)
        numSamples = 0;
        distArray = [];
        timeArray = [];
        poseArray = [];
        curvArray = [];
        vlArray = []
        vrArray = []
        VArray = [];
        wArray = [];
        V_max = 0.2;% Maximum Wheel Velocity
    end
        methods(Access = public)
            function obj = Trajectory_Null()
                %Implement this
            end
            
        end
end