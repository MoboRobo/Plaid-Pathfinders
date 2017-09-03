function line_rth = ParametrizeABC2Rth(line_abc)
% given a line as ax+by+c = 0
% return r,th such that xcos(th)+ysin(th) = r

a = line_abc(1); b = line_abc(2);
c = line_abc(3);

x = -(a*c)/(a^2+b^2);
y = -(b*c)/(a^2+b^2);
r = norm([x y]);
th = atan2(y,x);

line_rth = [r;th];
end

