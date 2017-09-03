function theta = atanLine2D(a,b)
%return angle of line of form ax + by + c = 0
theta = atan2(-a,b);
if a > 0
    theta = pi + theta;
end
if a == 0
    theta = 0;
end
if b == 0
    theta = pi/2;
end
end

