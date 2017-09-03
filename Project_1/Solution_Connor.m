% Solution for Problem of Robot Running Straight to a Point %targ_dist%
% meters away.
function Solution_Connor(rob, targ_dist)
 global DEBUG
 DEBUG = 1;
    
    PEAK_VELOCITY = 0.02; %m/s
    L2R_FACTOR = 1; %Ratio of Left-to-Right Wheel Running Speeds (Curvature Correction)
    
    rob.forksUp();
    rob.forksDown();
    
    debug_EncoderPosition(rob);
    i=0;
    while(i<1000)
        rob.sendVelocity(0.1,0.1);
        debug_EncoderPosition(rob);
        i = i+1;
    end
    
    
    while (avg_dist(rob) <= targ_dist)
        vel = PEAK_VELOCITY;
        if(avg_dist(rob) > targ_dist/2)
            vel = PEAK_VELOCITY * 2*(targ_dist - avg_dist(rob))/targ_dist;
        end
        
        runStraight(rob, vel,vel,L2R_FACTOR);
        
     pause(0.005)
    end
    rob.stop();
    clear_persistent();
end % #Solution_Connor

%Helper Function
%Compute Average Distance Traversed (by L&R encoders)
function ad = avg_dist(rob)
    ad = (rob.encoders.LatestMessage.Vector.X+rob.encoders.LatestMessage.Vector.Y)/2;
end % #avg_dist()

%Helper Function
%Calculate the Last Encoder Time Stamp for the Robot
function t = enc_time(rob)
    t = double(rob.encoders.LatestMessage.Header.Stamp.Sec) ...
        + double(rob.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
end % #enc_time

%Run Robot Straight with relative wheel velocities v_l, v_r, correcting for
%curvature defined by L2R_Factor (Ratio of Left-to-Right Wheel Running
%Speeds)
function runStraight(rob, v_l, v_r, l2r)
    rob.sendVelocity(v_l/l2r,v_r)
end % #runStraight

%Debugging Function
%Continuously Plot the Average Encoder Position
function debug_EncoderPosition(rob)
 global DEBUG
 
    persistent fig pl ts ds
    if isempty(fig) %Initialize Plots and Data (only once on first call)
        ts = zeros(1);
        ds = zeros(1);
        
        fig = figure();
        pl = plot(ts, ds);
            axis([0 60 0 50]);
            set(pl, 'XData', ts)
            set(pl, 'YData', ds)
            refreshdata
            drawnow
    end
    
    if DEBUG
        ts = [ts(1:end) enc_time(rob)]; % Time Data
        ds = [ds(1:end) avg_dist(rob)]; % Distance Data     
        set(pl, 'XData', ts)
        set(pl, 'YData', ds)
        refreshdata
        drawnow limitrate
    end % DEBUG
end % #debug_EncoderPosition

%Debugging Function
%Clears Persistent Variables from Debug Functions
function clear_persistent()
    clear debug_EncoderPosition
end