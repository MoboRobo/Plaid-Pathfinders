%% Makes a Null Instance of Trajectory_CubicSpiral (not a child)
function ntcs = MakeNullInstance_TCS()
    ntcs = Trajectory_CubicSpiral([0 0 0], 3);
    ntcs.is_null = 1;
end