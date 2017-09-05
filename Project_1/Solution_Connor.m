% Solution for Problem of Robot Running Straight to a Point %targ_dist%
% meters away.
function Solution_Connor(robot_id, targ_dist)
    %% SETUP ROBOT
    close all
    clear robot
    rob = raspbot(robot_id, [0; 0; pi/2])
    global init_l init_r
    init_l = rob.encoders.LatestMessage.Vector.X;
    init_r = rob.encoders.LatestMessage.Vector.Y;
    
    %% SETUP MAP
    % where robot starts at [0;0; -90]
    bounds = lineObject();
    bounds.lines = [-0.5 0; 0.5 0; 0.5 1; -0.5 1; -0.5 0];
    target = lineObject();
    target.lines = [-0.01 (targ_dist-0.01); 0.01 (targ_dist-0.01); 0.01 (targ_dist+0.01); ...
              -0.01 (targ_dist+0.01); -0.01 (targ_dist-0.01)];
    map = lineMap([bounds target]);
    hf = map.plot();
    if ishandle(hf)
        close(hf);
    end
    rob.genMap(map.objects);
    
    %% TASK PARAMETERS
    global DEBUG PEAK_SPEED MIN_SPEED L2R_FACTOR
    DEBUG = 1;
    
    PEAK_SPEED = 0.05; %m/s
    MIN_SPEED = 0.02; %m/s (minimum sustainable velocity)
    L2R_FACTOR = 1; %Ratio of Left-to-Right Wheel Running Speeds (Curvature Correction)
    
    %% PROCEDURE
    go_to(rob, targ_dist)
    pause(1)
    go_to(rob, 0)
    
    %% DEINITIALIZE
    rob.stop();
    rob.shutdown();
    clear_persistent();
end % #Solution_Connor


%% GOTO FUNCTION
%Motion Function
%Go to Specified One-Dimensional Position (with velocity curve)
function go_to(rob, pos)
    global init_l init_r
    global PEAK_SPEED MIN_SPEED L2R_FACTOR
    
    strt = avg_dist(rob, init_l,init_r); %starting position
    D_TS = pos - avg_dist(rob, init_l,init_r); %delta btwn starting and target position
    abs_dts = abs(D_TS);

    D_TC = pos - avg_dist(rob, init_l,init_r); %delta btwn Current and Target position
    abs_dtc = abs(D_TC);
    
    spd = PEAK_SPEED;
    while (spd ~= 0)
        ad = avg_dist(rob, init_l,init_r);

        D_CS = ad -strt; %delta btwn Start and Current Position
        abs_dcs = abs(D_CS);

        D_TC = pos - ad; %delta btwn Current and Target position
        abs_dtc = abs(D_TC);

        if(abs_dcs > abs_dts/2) %if past half-way
            spd = (PEAK_SPEED-MIN_SPEED) * abs_dtc/(abs_dts/2) + MIN_SPEED;
        end% if |DCS|>|DTS|/2
        if(abs_dcs > abs_dts) %if done.
            spd = 0;
        end % if |DCS|>|DTS|

        if(D_TS < 0) %Go Backwards
            [avg_dist(rob, init_l,init_r) pos -spd]
            runStraight(rob, -spd,-spd,L2R_FACTOR);
        else %Go Forward
            [avg_dist(rob, init_l,init_r) pos spd]
            runStraight(rob, spd,spd,L2R_FACTOR);
        end % D_TS<0

        pause(0.05)
        debug_EncoderPosition(rob);
    end %while(spd~=0)
end % #go_to


%% AVG-DISTANCE
%Helper Function
%Compute Average Elapsed Distance Traversed (by L&R encoders, subtracting
%initial left and right encoder positions: l_0, r_0 respectively)
function ad = avg_dist(rob, l_0,r_0)
    ad = (rob.encoders.LatestMessage.Vector.X-l_0 ...
        + rob.encoders.LatestMessage.Vector.Y-r_0)/2;
end % #avg_dist()


%% CURVATURE CORRECTION
%Run Robot Straight with relative wheel velocities v_l, v_r, correcting for
%curvature defined by L2R_Factor (Ratio of Left-to-Right Wheel Running
%Speeds)
function runStraight(rob, v_l, v_r, l2r)
    rob.sendVelocity(v_l/l2r,v_r)
end % #runStraight


%% PLOTTING
%Debugging Function
%Continuously Plot the Average Encoder Position
function debug_EncoderPosition(rob)
 global DEBUG init_l init_r
 
    persistent fig pl_avg pl_left pl_right ts ds ls rs
    if isempty(fig) %Initialize Plots and Data (only once on first call)
        tic
        ts = zeros(1);
        ds = zeros(1); % avg
        ls = zeros(1); % left enc
        rs = zeros(1); % right enc
        
        fig = figure();
        hold on
        pl_avg = plot(ds, '-k');
        pl_left = plot(ls, 'b');
        pl_right = plot(rs, 'r');
            ylim([0 0.4]) % <-MAX Y-VALUE
            legend('Average Reading', 'Left Encoder', 'Right Encoder')
            xlabel('Time Elapsed [s]')
            ylabel('Position, y [m]')
            set(pl_avg, 'YData', ds)
            set(pl_left, 'YData', ls)
            set(pl_right, 'YData', rs)
        hold off
        refreshdata
        drawnow
    end
    
    if DEBUG
        ts = [ts(1:end) toc]; % Time Data
        ds = [ds(1:end) avg_dist(rob, init_l, init_r)]; % Distance Data
        ls = [ls(1:end) (rob.encoders.LatestMessage.Vector.X-init_l)];
        rs = [rs(1:end) (rob.encoders.LatestMessage.Vector.Y-init_r)];
        hold on
        set(pl_avg, 'XData', ts)
        set(pl_left, 'XData', ts)
        set(pl_right, 'XData', ts)
        set(pl_avg, 'YData', ds)
        set(pl_left, 'YData', ls)
        set(pl_right, 'YData', rs)
        hold off
        refreshdata
        drawnow
    end % DEBUG
end % #debug_EncoderPosition

%Debugging Function
%Clears Persistent Variables from Debug Functions
function clear_persistent()
    clear debug_EncoderPosition
end