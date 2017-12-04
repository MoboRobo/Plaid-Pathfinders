function SelectionFilterTest(robot_id)
    units(); % Import Units
    global ft deg
    
    bounds = 2*[6*ft 0; 0 0; 0 -8*ft; 6*ft -8*ft]; % U with bottom left corner at origin
    wm = WorldMap(bounds);
    
    robot_starting_pose = pose(0.75*ft, -0.75*ft, -pi);
    
    rob = P2_Robot(raspbot(robot_id), robot_starting_pose, wm);
    rob.laserOn();
    
    pause(5);
    
    figA = figure();
    figB = figure();
    while(1)
        figure(figA);
        r_img = rob.hist_laser.last;
        
        r_img.plot(1);
        r_img.findLineCandidates();
        r_img.plotLineCandidates(0, gca);
        
        figure(figB);
%         r_img = RangeImage.select_Rel(rob.hist_laser.last, 1.5*0.07, -0.23, 1.9);
        r_img = RangeImage.select(rob.hist_laser.last, 1.5*0.07, 2*ft, -7*ft);
        
        r_img.plot(1);
        r_img.findLineCandidates();
        r_img.plotLineCandidates(0, gca);
        pause % Wait for Terminal Key
    end % loop
    
end % #Lab13