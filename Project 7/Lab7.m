function Lab7(robot_id)
%     mrplt = mrplTestbox(robot_id);
%     mrplt.test();
     mrpl = mrplSystem(robot_id, pose(0,0,0));
     
     mrpl.goTo_Rel( pose(0.3048,0.3048,0.0) );
     mrpl.goTo_Rel( pose(-0.6096,-0.6096,-pi/2.0) );
     mrpl.goTo_Rel( pose(-0.3048,0.3048,pi/2.0) );
     
     pause % Avoid GC
end