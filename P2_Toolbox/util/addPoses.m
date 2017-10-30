function p = addPoses(p1, p2)
    x = p1.x + p2.x;
    y = p1.y + p2.y;
    th = p1.th + p2.th;
    th = atan2(sin(th), cos(th));
    p = pose(x, y, th);
end