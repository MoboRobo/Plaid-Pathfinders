maxIterations = 15;
p1 = [-2 ; -2];
p2 = [ 2 ; -2];
p3 = [ 2 ; 2];
p4 = [-2 ; 2];
lines_p1 = [p1 p2 p3 p4];
lines_p2 = [p2 p3 p4 p1];
% Set up test points
nPts = 10;
x1 = -2.0*ones(1,nPts);
x2 = linspace(-2.0,2.0,nPts);
x3 = 2.0*ones(1,nPts);
y1 = linspace(0.0,2.0,nPts);
y2 = 2.0*ones(1,nPts);
y3 = linspace(2.0,0,nPts);
w = ones(1,3*nPts);
x1pts = [x1 x2 x3];
y1pts = [y1 y2 y3];
w1pts = w;


modelPts = [x1pts ; y1pts ; w1pts]
size(modelPts)
% pick a pose
dx = -0.05*rand();
dy = -0.05*rand();
dt = -0.05+0.2*rand();
thePose = pose(0.0+dx,0.0+dy,0.0+dt);
%  thePose = pose(1, 1, 2);


myObj = lineMapLocalizer(lines_p1, lines_p2, .01, .001, .0005);
tic()
myObj.refinePose(thePose, modelPts, 40);
toc()
