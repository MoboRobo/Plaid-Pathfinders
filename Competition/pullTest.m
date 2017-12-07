%% Good Robots:
% - RaspBot 30
% - RaspBot 35?

function pullTest(robot_id, spd)
global ri

    units();
    global ft;
    
    speed = spd;
    bounds = [8*ft 0; 0 0; 0 -8*ft; 8*ft -8*ft]; % U with bottom left corner at origin
    wm = WorldMap(bounds);
    
    
    robot_starting_pose = pose(1*ft, -1*ft, -pi);
    
    ri = struct('mrpl',mrplSystem.empty);
    ri.mrpl = mrplSystem(robot_id, robot_starting_pose, wm);
    
    pause(5);
    disp('Ready. . .');
    pause
    
    p_stash_test = pose(3*ft, -2*ft, 0);
    p_stash_test_pull = pose(1*ft, -2*ft, 0);
    
    p_stash_test2 = pose(3*ft, -3*ft, 0);
    p_stash_test_pull2 = pose(1*ft, -3*ft, 0);
    
    pull(p_stash_test, speed, p_stash_test_pull);
    pull(p_stash_test2, speed, p_stash_test_pull2);

end % #pullTest

% Pull from pose p at speed spd to the stash point dist.
function pull(pa, spd, p_stash)
global ri;
    ri.mrpl.rob.core.forksDown();

    offset_seed = pose(-0.055, 0, 0); % Actual Fork Depth 4cm.
    offset_acq = pose(-0.35, 0.1, 0);
        
    p_seed = addPoses(p_stash, offset_seed);
    p_acq = addPoses(pa, offset_acq);
    
    pa.poseVec
    p_stash.poseVec
    p_seed.poseVec
    
    ri.mrpl.face(p_seed);
    ri.mrpl.face(p_seed);
    ri.mrpl.goTo_X_Small(ri.mrpl.rob.measTraj.p_f.Y - p_seed.Y, 0.75*spd); % Fast Ortho.
    
    ri.mrpl.face(p_acq);
    ri.mrpl.face(p_acq);
    ri.mrpl.goTo(p_acq, spd);
    
                                                                            pause
                                                                            beep
    
    [present, p_nlo_w, p_nlo_r] = ri.mrpl.lookForPalletNear(pa.X,pa.Y, 0.25);
    if(present)
                                                                            disp(p_nlo_w.poseVec);
                                                                            beep
                                                                            pause
        ri.mrpl.face(p_nlo_w);
        ri.mrpl.face(p_nlo_w);
        
        ri.mrpl.goTo(p_nlo_w, 0.09);

        ri.mrpl.rob.core.forksUp();
        pause(0.5); % Get Forks Up
        
        ri.mrpl.goTo_X_Small(p_stash.X - ri.mrpl.rob.measTraj.p_f.X, spd);

        ri.mrpl.rob.core.forksDown();

                                                                                pause
        
        ri.mrpl.goTo_X_Small(p_seed.X - ri.mrpl.rob.measTraj.p_f.X, 0.5*spd); % Get this right (localization-wise)
    
        ri.mrpl.turn_stationary(-pi/2);
    end
    
end % #pull