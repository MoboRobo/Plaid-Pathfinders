
function Lab4_Ben(robot_id)
%% SETUP ROBOT
    rasp = raspbot(robot_id, [0; 0; pi/2])
    rob = P2_Robot(rasp);
    if(~strcmp(robot_id,'sim'))
        rob_type = 'raspbot';
        rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
        RangeImage.INDEX_OFFSET(5);
        rob.core.forksDown(); % Prevent Brown-out
    end

    %% SETUP MAPPING
    bounds = 3.5*[0.5 0; 0.5 1; -0.5 1; -0.5 0]; % Inverted U-Shaped Container
    person_lines = ShapeGen.rect(0.1,0.05);
    wm = WorldMap(rob, bounds);
        person = wm.addObstacle(person_lines); % Person to Follow.
        person.pose = [0.25 0.5 0]; % Move to Center Screen
    wm.createMap();
    pause(1) % Wait for World to Initialize
    
    %% PLOT
    rob.logEncoders();
    figure;
    p = plot(-rob.hist_y,rob.hist_x);
    t0 = tic
    vmax = 0.25;
    amax = 3*0.25;
    dist = 1.0;
    while(toc(t0) < 20)
        rob.moveAt(trapezoidalVelocityProfile(toc(t0) ,amax,vmax,dist),0)
        set(p, 'xdata', [get(p,'xdata') -rob.hist_y(end)], 'ydata', [get(p,'ydata') rob.hist_x(end)]);
        refreshdata
        pause(0.05);
    end
    rob.moveAt(0,0)
end

function uref = trapezoidalVelocityProfile( t , amax, vmax, dist)
    if(t < 0)
        uref = 0;
    end
    tRamp = amax/vmax;
    tF = dist/vmax + vmax/amax;
    if(t < tRamp)
        fprintf('Ramping up')
        uref = amax*t;  
    elseif (tF - t > 0 && tF - t < tRamp)
        fprintf('Ramping down')
        uref = amax*(tF-t);
    elseif(tRamp <= t && t <= tF- t)
        fprintf('Constant Speed')
        uref = vmax;
    else
        uref = 0;
    end
end