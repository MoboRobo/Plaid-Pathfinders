function Lab9(robot_id)
    mrpl = mrplSystem(robot_id, pose(0,0,0));
     
    %% SETUP MAPPING
    if(strcmp(robot_id,'sim'))
        bounds = 2*[0.5 0; 0.5 1; -0.5 1; -0.5 0]; % Inverted U-Shaped Container
        block = ShapeGen.rect(0.038,0.127);
        wm = WorldMap(mrpl.rob, bounds);
            % Create a Ring of Blocks around Origin.
            r_min = 0.42;
            r_max = 1;
            n = 4;
            ths = (0 : pi/2/n : pi/2 + pi/2/n); % Go one block beyond first quadrant
            for th = ths
                obs = wm.addObstacle(block);
                r_rel = r_min + (r_max-r_min) * (th / (pi/2)); % Create Spiral
                obs.pose = [r_rel*cos(th) r_rel*sin(th) th];
            end
        wm.createMap();
    end
     
    mrpl.plottingOn(); % Turn on Plotting

    pause(2); % Wait for Robot to Initialize
    forkDistance = .04;
    overdrive = 0.02;
    spacing = .07;
    for i = 1:3
        p_nlo_r = mrpl.getNearestLineObject();

        p_acq_r = mrpl.acquisitionPose(p_nlo_r, 0.067, 0.02, spacing, 0.01);

        %go to spacing distance before the sail
        mrpl.goTo_Rel( p_acq_r );

        %then move forward and pick up the sail, with overdrive
        mrpl.goTo_X_Small(spacing + overdrive);
        mrpl.rob.core.forksUp();
        % wait a single second then drop the forks
        pause(1);
        mrpl.rob.core.forksDown();
        %wait another second and move exactly one fork length away
        mrpl.goTo_X_Small(spacing + overdrive);
        %rotate 180
        mrpl.turn_stationary(pi);
        pause(10);
    end
     
     pause % Avoid GC
end