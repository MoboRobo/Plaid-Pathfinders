%% Initialize.
% run ../initLocal.m;
close all; clc;

%% Setup world.
% lineObject instances represent bodies.
walls = lineObject();
% Specify points as an n x 2 array.
% Simulator length units: m.
walls.lines = [1.5 0; 3 1.5; 1.5 3; 0 1.5; 1.5 0];
obstacle = lineObject();
obstacle.lines = [1.6 1.4; 1.6 1.6; 1.4 1.6; 1.4 1.4; 1.6 1.4];

% Environments are specified as lineMap instances.
% lineMap constructor takes an array of lineObject instances as input.
map = lineMap([walls obstacle]);
% Show the map.
hf = map.plot();

%% Fire simulator.
% Close map plot.
if ishandle(hf)
    close(hf);
end

% By default robot starts at pose x = 0, y = 0, theta = 0.
%rob = raspbot('sim');
% Start at desired pose.
rob = raspbot('sim',[1; 1.5; 0]);


%% Add map to simulator.
% neato.genMap takes an array of lineObject instances as input. 
rob.genMap(map.objects);

%% Move robot.
% Simulator does not check for collisions.
rob.sendVelocity(0.1,0.1);

%% Teleport robot.
% A shortcut to get robot where you want.
rob.sim_robot.pose = [1; 1.5; 0];

%% Start laser.
% Need to call neato.genMap() before starting laser.
rob.startLaser();
pause(0.1);
% Try accessing ranges via rob.laser.data.ranges
% Try accessing encoders via rob.encoders.data

%% Stop Laser.
rob.stopLaser();
pause(0.1);
% Toggle plot with rob.togglePlot();

%% Kill simulator.
% Simulator runs MATLAB timer objects. Calling neato.shutdown() kills
% simulator cleanly.
rob.shutdown();
% If you deleted the robot without neato.shutdown(), run
% delete(timerfindall) to get rid of timers.

%% Clean workspace.
close all; clear all; clc;    