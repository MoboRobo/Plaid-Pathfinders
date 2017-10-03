function params = ParametrizePts2ABC(p1,p2)
% given points p1 and p2, return line joining them in form ax+by+c=0
% params = [a;b;c];

if p1(1) == p2(1)
    params = [1;0;-p1(1)];
elseif p1(2) == p2(2)
    params = [0;1;-p1(2)];
else
    [m,c] = ParametrizePts2MC(p1,p2);
    params = [m;-1;c];
end

end

