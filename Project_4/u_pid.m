function control = u_pid(t, goalDistance, distanceTraveled)
    global rob lastError errorIntegral
    v_max = .3
    k_p = 10.0;
    k_d = 0.0;
    k_i = 0.0;
    maxErrorIntegral = 10;
    error = goalDistance - distanceTraveled
    dt = t - rob.hist_commTime(end)
    errorDerivative = (error-lastError)/ dt
    errorIntegral = errorIntegral + error*dt
    if abs(errorIntegral) > maxErrorIntegral
        errorIntegral= (errorIntegral/abs(errorIntegral))*maxErrorIntegral
    end
    
    
    control = k_p * error + k_d * errorDerivative + k_i * errorIntegral
    
    if abs(control) > v_max
        control = (control / abs(control)) * v_max
    end
    lastError = error
end