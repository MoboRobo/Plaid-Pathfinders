function [m,c] = ParametrizePts2MC(p1,p2)
%given points p1 and p2, return line joining them in form y = mx + c

m = (p2(2)-p1(2))/(p2(1)-p1(1));
c = p1(2)-m*p1(1);

end