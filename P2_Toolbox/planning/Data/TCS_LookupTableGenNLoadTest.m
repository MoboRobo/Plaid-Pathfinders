% Tests Generating and Loading a TCS Data Lookup Table of Scale, scale, 
% from a *.mat File.
function TCS_LookupTableGenNLoadTest(scale)
    Trajectory_CubicSpiral.makeLookupTable(scale);
    pause(1); %Relief.
    TCS = Trajectory_CubicSpiral.planTrajectory(0.5,0.6,pi, 1, 201,scale);
    
    figure();
    TCS.plot();
end