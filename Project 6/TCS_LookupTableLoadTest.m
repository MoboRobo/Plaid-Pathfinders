% Tests Loading a TCS Data Lookup Table of Scale, scale, from a *.mat File.
function TCS_LookupTableLoadTest(scale)
    TCS = Trajectory_CubicSpiral.planTrajectory(0.1,0.2,pi/2, 1, 201,scale);
    
    figure();
    TCS.plot();
end