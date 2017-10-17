function TurnInPlaceTest(robot_id)
%     mrplt = mrplTestbox(robot_id);
%     mrplt.test();
     mrpl = mrplSystem(robot_id, pose(0,0,0));
     mrpl.plottingOn();
     
end
