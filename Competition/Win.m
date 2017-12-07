% Project GitHub (Pull, Stash, Push)
function Win(robot_id, spd)
global ri

    units(); % Import Units
    global ft
    
    speed = spd;
    bounds = [8*ft 0; 0 0; 0 -8*ft; 8*ft -8*ft]; % U with bottom left corner at origin
    wm = WorldMap(bounds);
    
    robot_starting_pose = pose(1*ft, -1*ft, -pi);
    
    ri = struct('mrpl',mrplSystem.empty);
    ri.mrpl = mrplSystem(robot_id, robot_starting_pose, wm);
   % ri = RobotInterface(robot_id, robot_starting_pose, wm);
    
    [pullSites, dropOffs] = generateSites(0); % 0 -> Test, 1-> Real %%%%%%%%%%%%% * $*$**$(*%(Q#*$(#*$ *()@#&%)$*( #
   
    pause(7); % REALLY Ensure Robot Has Stable Localization before Beginnning
    
    disp('Ready . . .');
    pause % WAIT FOR KEY.
    
    
%     ri.mrpl.goTo_th_Small(pi);
%     ri.mrpl.goTo(pose(2*ft, -1*ft, 0), speed); % Localize while moving to get better view.
%     mrpl.rob.core.play('FCd.wav');
    last_failed = 0; % Whether last pickup failed
    while (length(pullSites) > 0)
        nextPickup = pullSites(1);
        stashLocation = dropOffs(1);
        
        offset_seed = pose(-0.055, 0, 0); % Actual Fork Depth 4cm.
        p_seed = addPoses(nextPickup, offset_seed);
        
        if nextPickup.X > 5*ft % In the line of pallets
        % Do Seed, Pull, Stash Routine.
    %         ri.mrpl.pickupObjAt(nextPickup, speed)
            fail = pull(nextPickup, speed, stashLocation, ~last_failed);
            last_failed = 0;

            pause(0.5); % Wait to /see/ if pallet has tipped.

            if ~fail
                if (ri.mrpl.rob.hist_laser.last.isObscured()) % Pull successful
                    nextDropoff = dropOffs(1);
                    dropOffs = dropOffs(2:end);
                end

                ri.mrpl.goTo_X_Small(p_seed.X - ri.mrpl.rob.measTraj.p_f.X, 0.5*spd); % Get this right (localization-wise)

                ri.mrpl.turn_stationary(-pi/2);
            else
                % Failure is returned when near next pallet, so just go there
                last_failed = 1; % Signal this.
            end % fail?

            pullSites = pullSites(2:end);
        else
        % Side Pallet, do Lab13 Routine.
            nextPickup.poseVec
            ri.mrpl.pickupObjAt(nextPickup, speed)
            pause(0.5); % Wait to /see/ if pallet has tipped.
            if (ri.mrpl.rob.hist_laser.last.isObscured()) % is holding pallet?
                nextDropoff = dropOffs(1);
                ri.mrpl.dropObjAt(nextDropoff, min(speed*1.25,0.45));
                dropOffs = dropOffs(2:end);
                ri.mrpl.goTo_X_Small(-0.1, 0.15); % Extra back-up
            else
                Dx = abs(ri.mrpl.rob.measTraj.p_f.X - nextPickup.X) + 0.5;
                ri.mrpl.goTo_X_Small(-Dx, 0.35); %Back up in shame after failure.

                pause(0.75); % Relocalize.
            end
            pullSites = pullSites(2:end);
        end % nextPickup.X?
        
    end % while
    
    pause; % Avoid GC.
    
end % #Lab13


% Pull from pose p at speed spd to the stash point dist.
function fail = pull(pa, spd, p_stash, go_to_seed)
global ri;
    fail = 0;

    ri.mrpl.rob.core.forksDown();

    offset_seed = pose(-0.055, 0, 0); % Actual Fork Depth 4cm.
    offset_acq = pose(-0.35, 0, 0);
        
    p_seed = addPoses(p_stash, offset_seed);
    p_acq = addPoses(pa, offset_acq);
    
    pa.poseVec
    p_stash.poseVec
    p_seed.poseVec
    
    if go_to_seed
        ri.mrpl.face(p_seed);
        ri.mrpl.face(p_seed);
        ri.mrpl.goTo_X_Small(ri.mrpl.rob.measTraj.p_f.Y - p_seed.Y, 0.75*spd); % Fast Ortho.
    end
    
    ri.mrpl.face(p_acq);
    ri.mrpl.face(p_acq);
    ri.mrpl.goTo(p_acq, spd);
    
    [present, p_nlo_w, p_nlo_r] = ri.mrpl.lookForPalletNear(pa.X,pa.Y, 0.25);
    if(present)
        ri.mrpl.face(p_nlo_w);
        ri.mrpl.face(p_nlo_w);
        
        ri.mrpl.goTo(p_nlo_w, 0.09);
        
        ri.mrpl.rob.core.forksUp();
        pause(0.5); % Get Forks Up
        
        if(ri.mrpl.rob.hist_laser.last.isObscured()) % Has Pallet
            ri.mrpl.goTo_X_Small(p_stash.X - ri.mrpl.rob.measTraj.p_f.X, spd);

            ri.mrpl.rob.core.forksDown();
        else
            fail = 1;
        end
    else
        fail = 1;
    end
    
end % #pull