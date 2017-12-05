function SelectionFilterTest(robot_id)
    units(); % Import Units
    global ft
    
    bounds = [7*ft 0; 0 0; 0 -8*ft; 7*ft -8*ft]; % U with bottom left corner at origin
    wm = WorldMap(bounds);
    
    robot_starting_pose = pose(0.75*ft, -0.75*ft, -pi);
    
    mrpl = mrplSystem(robot_id, robot_starting_pose, wm);
    mrpl.rob.laserOn();
    
    pause(5);
    
    figA = figure(); % Capture Test.
    figB = figure(); % Selection Window Test.
    figC = figure(); % Presence Test.
    while(1)
        % Analyse Whole Capture:
        figure(figA);
        r_img = mrpl.rob.hist_laser.last;
        
        r_img.plot(1);
        r_img.findLineCandidates();
        r_img.plotLineCandidates(0, gca);
        
        % Analyse Selection Window Around a Pallet:
        figure(figB);
%         r_img = RangeImage.select_Rel(rob.hist_laser.last, 1.5*0.07, -0.23, 1.9);
        r_img = RangeImage.select(mrpl.rob.hist_laser.last, 1.5*0.07, 6*ft, -1*ft);
        
        r_img.plot(1);
        r_img.findLineCandidates();
        r_img.plotLineCandidates(0, gca);
        
        % Check for All Pallets (in the row, not the side):
        capture_img = mrpl.rob.hist_laser.last; % Operate all on same capture
        presents = [];
        figure(figC);
        xa = 6*ft;
        for i=1:7
            disp(i);
            ya = -i*ft;
            
            [present, p_w] = mrpl.lookForPalletNear(xa,ya);
            presents(end+1) = present
            
            r_img = RangeImage.select(capture_img, 1.5*0.07, xa, ya);
            
            r_img.plot(1);
            r_img.findLineCandidates();
            r_img.plotLineCandidates(0, gca);
            pause % Display selection window for a sec.
        end % i=1:7
        
        disp(presents);
        
        pause % Wait for Terminal Key
    end % loop
    
end % #Lab13