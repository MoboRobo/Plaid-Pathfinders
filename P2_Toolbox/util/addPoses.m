% Adds Pose 2 relative to Pose 1
function p = addPoses(p1, p2)
    p2w = Trajectory.poseToWorld(p2, p1);
    p = p2w;
%     x = p1.x + p2w.x;
%     y = p1.y + p2w.y;
%     th = asum(p1.th, p2w.th);
%     p = pose(x, y, th);
end