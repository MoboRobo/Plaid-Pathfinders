function Lab13(robot_id, spd)
    units(); % Import Units
    global ft
    
    speed = spd;
    bounds = [8*ft 0; 0 0; 0 -8*ft; 8*ft -8*ft; 8*ft 0]; % Box with bottom left corner at origin
    wm = WorldMap(bounds);
    
    robot_starting_pose = pose(1*ft, -1*ft, -pi);
    
    ri = struct('mrpl',mrplSystem.empty);
    ri.mrpl = mrplSystem(robot_id, robot_starting_pose, wm);
   % ri = RobotInterface(robot_id, robot_starting_pose, wm);
   
   [pickups, dropOffs] = generateSites(0); % 0 -> Test, 1-> Real %%%%%%%%%%%%% * $*$**$(*%(Q#*$(#*$ *()@#&%)$*( #

%     startDropY = -1.5*ft; dropYIncrement = -.75*ft;
%     pickups = [
%         pose(6*ft, -2*ft, 0),...
%         pose(6*ft, -3*ft, 0), pose(6*ft, -4*ft, 0), ...
%         pose(6*ft, -5*ft, 0),...
%         pose(2*ft, -7*ft, -pi/2),...
%         pose(3*ft, -7*ft, -pi/2),...
%         pose(4*ft, -7*ft, -pi/2),...
%         pose(6*ft, -6*ft, 0),...
%         pose(6*ft, -7*ft, 0),...
%         pose(6*ft, -1*ft, 0),];
%     pickups = [pickups pickups]; % Double List
%         
%     dropOffs = [
%         pose(1*ft+0.06, startDropY, pi), ... 
%         pose(1*ft+0.06, startDropY +1*dropYIncrement, pi), ...
%         pose(1*ft+0.06, startDropY +2*dropYIncrement, pi),...
%         pose(1*ft+0.06, startDropY +3*dropYIncrement, pi),...
%         pose(1*ft+0.06, startDropY +4*dropYIncrement, pi),...
%         pose(1*ft+0.06, startDropY +5*dropYIncrement, pi),...
%         pose(1*ft+0.06, startDropY +6*dropYIncrement, pi),];
%     dropOffs = [dropOffs dropOffs]; % Double List
    
    pause(7); % REALLY Ensure Robot Has Stable Localization before Beginnning
    
    disp('Ready . . .');
    pause % WAIT FOR KEY.
    
%     ri.mrpl.goTo_th_Small(pi);
%     ri.mrpl.goTo(pose(2*ft, -1*ft, 0), speed); % Localize while moving to get better view.
%     mrpl.rob.core.play('FCd.wav');
    while (length(pickups) > 0)
        nextPickup = pickups(1);
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
            ri.mrpl.goTo_X_Small(-Dx, 0.4); %Back up in shame after failure.
                
            pause(0.75); % Relocalize.
        end
        pickups = pickups(2:end);
    end
    
    pause; % Avoid GC.
    
end % #Lab13