function FixDatProblem()
    p_rob = ProbRob();
    rob = P2_Robot(raspbot('sim'));
    clk = Clock();
    
    V_norm = 0.1;
    rho = 0.5;
    
    t = 0;
    while(t < 10)
        x = p_rob.test_prop.data_A.last.poseVec;
        
        p_rob.updateTheThing_Callbackesque();
        
        t = clk.time();
        evnt = struct( ...
        'Vector', struct('X',0.1*clk.time(), 'Y',0.2*clk.time()), ...
        'Header', struct('Stamp', struct('Sec',clk.time(), 'Nsec',0)) ...
        );
        %rob.encTraj.update(V_norm*t, V_norm*t/rho,clk.time());
        rob.processNewEncoderData(0, evnt);

        y = rob.measTraj.data_poses.last.poseVec
        
        pause(0.01);
    end
    rob.moveAt(0,0);
    
    figure();
    rob.measTraj.plot();
end