function om_limits_test()
    %% SETUP ROBOT
    rasp = raspbot(robot_id, [0; 0; pi/2])
    rob = P2_Robot(rasp);
    if(~strcmp(robot_id,'sim'))
        rob_type = 'raspbot';
        rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
        RangeImage.INDEX_OFFSET(5);
        rob.core.forksDown(); % Prevent Brown-out
    end
    
    clk = Clock();
    
    %% TESTING
    res = 0.1;
    oms = (0:res:2*pi);
    
    figure();
    hold on
        pl_ref = PersistentPlot(0,0);
        pl_meas = PersistentPlot(0,0);
    hold off
    
    i = 1;
    while (i < length(oms))
       rob.moveAt(0,oms(i));
       
       pause(0.5);
       
       pl_ref.addXY((i-1)*res, oms(i));
       pl_meas.addXY((i-1)*res, rob.hist_estVel(end).om);
       
        
    i = i+1;
    end

end