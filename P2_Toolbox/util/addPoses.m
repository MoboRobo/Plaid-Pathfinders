% Adds Pose 2 relative to Pose 1
function p = addPoses(p_world, p_rel)
    pw = Trajectory.poseToWorld(p_rel, p_world);
    p = pw;
end