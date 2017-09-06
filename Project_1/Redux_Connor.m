% Solution for Problem of Robot Running Straight to a Point %targ_dist%
% meters away.
% Solution simplified by using P2 API/Toolbox.
% Example: Redux_Connor('sim', 0.3)
function Redux_Connor(robot_id, targ_dist)
    global debugger % Allow debugger instance to be called within other functions
    
    %% SETUP ROBOT
    rasp = raspbot(robot_id, [0; 0; pi/2])
    rob = P2_Robot(rasp);
    if(~strcmp(robot_id,'sim'))
        rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
    end
    
    %% SETUP MAPPING
    bounds = [-0.5 0; 0.5 0; 0.5 1; -0.5 1; -0.5 0];
    target = [-0.01 (targ_dist-0.01); 0.01 (targ_dist-0.01); ...
               0.01 (targ_dist+0.01); -0.01 (targ_dist+0.01); ...
              -0.01 (targ_dist-0.01)];
          
    wm = WorldMap(rob, bounds);
     wm.addFlag(target);
    
    %% ACTIVATE DEBUGGING
    debugger = Debugger(rob);
     debugger.turnOn(); %Switch this to off to close all debugging features/plots
    
    %% TASK PARAMETERS
    rob.MIN_SPEED = 0.02; %m/s (minimum sustainable velocity)
    
    %% ALGORITHM
    rob.goTo_straight(targ_dist, 0.05);
    pause
    
    %% SHUTDOWN ROBOTS & FREE MEMORY
    rasp.shutdown();
    
    clear rob
    clear rasp
    clear robot
    close all
end