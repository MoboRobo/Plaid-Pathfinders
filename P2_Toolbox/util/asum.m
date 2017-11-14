% Returns the Angular Sum of angle1 and angle2.
function Sth = asum(a1,a2)
    Sth = a1 + a2;
    Sth = atan2(sin(Sth),cos(Sth));
end % #adel