function Lab7(robot_id)
    mrpl = mrplSystem(robot_id);
    mrpl.plottingOn();
    
    mrpl.goTo_Rel(0.3048,0.3048,0.0);
    mrpl.goTo_Rel(-0.6096,-0.6096,-pi/2.0);
    mrpl.goTo_Rel(-0.3048,0.3048,pi/2.0);
end