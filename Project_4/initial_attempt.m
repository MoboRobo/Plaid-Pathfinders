function moveStraightLine(goalDistance, robName)
    global rob lastError errorIntegral
    lastError = 0;
    errorIntegral = 0;
    rasp = raspbot(robName);
    rob = P2_Robot(rasp);
    maxTime = 6.0;
    startTime = rob.tripTime();
    timeCoords = [];
    distanceCoords = [];
    d = 0; t = 0;
    while ( d < goalDistance  && (t-startTime) < maxTime)
        t = rob.tripTime();
        d = rob.avgTripDist();
        control = u_pid(t, goalDistance, d)
        rob.core.sendVelocity(control, control);
        plot([rob.hist_enc(:).s_l], rob.hist_commTime)
        pause(.001)
    end
    rob.core.stop()
end