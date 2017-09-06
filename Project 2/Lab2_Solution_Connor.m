% Solution for Problem of Robot Running Straight to a Point %targ_dist%
% meters away.
function Lab2_Solution_Connor(robot_id, targ_dist)
    global debugger
    close all
    clear robot
    clear rob
    rasp = raspbot(robot_id, [0; 0; pi/2])
    rob = P2_Robot(rasp);
    if(~strcmp(robot_id,'sim'))
        rob.togglePlot(); %Turn on map plotting for non-simulated robots
    end
    
    bounds = [-0.5 0; 0.5 0; 0.5 1; -0.5 1; -0.5 0];
    wm = WorldMap(rob, bounds);
    new_flag = [-0.01 (targ_dist-0.01); 0.01 (targ_dist-0.01); 0.01 (targ_dist+0.01); ...
              -0.01 (targ_dist+0.01); -0.01 (targ_dist-0.01)];
    wm.addFlag(new_flag);
    
    debugger = Debugger(rob);
        db.turnOn();
        db.plot_encoderPosition(0.4);
end