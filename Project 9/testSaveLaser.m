function testSaveLaser(robot_id)
  mrpl = mrplSystem(robot_id, pose(0,0,0)); 
  clk = Clock();
  first_loop = 1;
  T = 0;
  
  pause(1);
  images = RangeImage.empty();
  
  counter = 1;
  while (T < 10)
    if(first_loop)
        clk = Clock();
        first_loop = 0;
        images = mrpl.rob.hist_laser.last();
    else
        images(end+1) = mrpl.rob.hist_laser.last();
    end
    T = clk.time();
  end
  save(strcat('rangeImages2'),'images');
    
end