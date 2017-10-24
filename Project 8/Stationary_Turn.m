function Stationary_Turn(rob, target_th)
  kp = 1.2;
  max_omega = 4; 
  
  curr_th = rob.measTraj.getFinalPose.th;
  clk = Clock();
  first_loop = 1;
  T = 0;
  
  pause(1);
  error = atan2(sin(curr_th - target_th), cos(curr_th - target_th))
  
  while (abs(error) > 0.01)
    if(first_loop)
        clk = Clock();
        first_loop = 0;
    end
    T = clk.time();
    curr_th = rob.measTraj.getFinalPose.th;
    error = atan2(sin(curr_th - target_th), cos(curr_th - target_th))
    pGain = -error*kp;
    if(pGain > max_omega)
       rob.moveAt(0, max_omega);
    elseif(pGain < -max_omega)
       rob.moveAt(0, -max_omega);
    else
       rob.moveAt(0, -error*kp);
    end
    pause(0.01);
  end
  rob.moveAt(0, 0);
end

           
           
           
