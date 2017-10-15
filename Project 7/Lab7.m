function Lab7(robot_id)
    %mrplt = mrplTestbox(robot_id);
    %mrplt.test();
     mrpl = mrplSystem(robot_id, [0,0,pi/2]);
     mrpl.plottingOn();
     
     mrpl.goTo_Rel(0.3048,0.3048,0.0);
     mrpl.goTo_Rel(-0.6096,-0.6096,-pi/2.0);
     mrpl.goTo_Rel(-0.3048,0.3048,pi/2.0);
end