function res = peekRangeImage()
%peekRangeImage Sees if the lidarcallback says there is new data. Does not
%wait for data.
%   To use this, execute the following line:
%   rh = event.listener(robot.laser,'OnMessageReceived',@neatoImageEventListener);
%   before calling this function.
    global laserDataReady;

    laserDataReadyLast = laserDataReady;
    laserDataReady = 0;
    if(laserDataReadyLast)
        res = true;
    else
        res = false;
    end
end