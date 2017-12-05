function Lab13(robot_id, spd)
    units(); % Import Units
    global ft
    
    speed = spd;
    bounds = [7*ft 0; 0 0; 0 -8*ft; 7*ft -8*ft]; % U with bottom left corner at origin
    wm = WorldMap(bounds);
    
    robot_starting_pose = pose(0.75*ft, -0.75*ft, -pi);
    
    ri = struct('mrpl',mrplSystem.empty);
    ri.mrpl = mrplSystem(robot_id, robot_starting_pose, wm);
   % ri = RobotInterface(robot_id, robot_starting_pose, wm);
    startDropY = -1.5*ft; dropYincrement = -.75*ft;
    pickups = [
        pose(6*ft, -1*ft, 0), pose(6*ft, -2*ft, 0),...
        pose(6*ft, -3*ft, 0), pose(6*ft, -4*ft, 0), ...
        pose(6*ft, -5*ft, 0), pose(6*ft, -6*ft, 0),...
        pose(6*ft, -7*ft, 0), pose(4*ft, -7*ft, -pi/2),...
        pose(3*ft, -7*ft, -pi/2), pose(2*ft, -7*ft, -pi/2)]
        
    
    
    
    
    dropOffs = [
        pose(1*ft+0.06, startDropY, pi), speed ), 
        pose(1*ft+0.06, startDropY +dropYIncrement, pi), 
        pose(1*ft+0.06, startDropY+2*dropYIncrement, pi),...
        pose(1*ft+0.06, startDropY+3*dropYIncrement, pi),...
        pose(1*ft+0.06, startDropY+4*dropYIncrement, pi),...
        pose(1*ft+0.06, startDropY +5*dropYIncrement, pi),...
        pose(1*ft+0.06, startDropY+6*dropYIncrement, pi)]
    
    pause(3); % Ensure Robot Has Stable Localization before Beginnning
    
    while (length(dropOffs) > 0)
        nextPickup = pickups(1);
        ri.mrpl.pickupObjAt(nextPickup, speed)
        if (ri.mrpl.rob.hist_laser.last.isObscured()) % is holding pallet?
            nextDropoff = dropOffs(1);
            ri.mrpl.dropObjAt(nextDropoff, speed);
            dropOffs = dropOffs(2:end);
        end
        nextPickup = pickups(2:end);
    end
    
end % #Lab13