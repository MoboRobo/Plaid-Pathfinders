% Returns the Angular Sum of angle1 and angle2.
function Dth = adel(a1,a2)
    Dth = a1 - a2;
    Dth = atan2(sin(Dth),cos(Dth));
end % #adel