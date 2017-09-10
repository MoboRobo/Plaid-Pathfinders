function Lab2_Solution_Connor(robot_id)
    global rob_type laser_data_avail person
    rob_type = 'sim';
    %% SETUP ROBOT
    rasp = raspbot(robot_id, [0; 0; pi/2])
    rob = P2_Robot(rasp);
    if(~strcmp(robot_id,'sim'))
        rob_type = 'raspbot';
        rob.core.togglePlot(); %Turn on map plotting for non-simulated robots
    end

    %% SETUP MAPPING
    bounds = [1 0; 1 2; -1 2; -1 0]; % Inverted U-Shaped Container
    person_lines = ShapeGen.rect(0.1,0.1);
    wm = WorldMap(rob, bounds);
        person = wm.addObstacle(person_lines); % Person to Follow.
        person.pose = [0 0.5 0]; % Move to Center Screen
    wm.createMap();
    pause(1) % Wait for World to Initialize
    
    %% SETUP SENSING
    rob.core.startLaser();
    rob.core.laser.NewMessageFcn=@new_rangeData; % Setup Callback to Process New Data
    laser_data_avail = 0; % Flag for New Unprocessed Laser Data 
        
    rob.core.sendVelocity(0.05, 0.05);
    
    %% INITIALIZE DATA PLOTTING
    fig_target = figure();
    plot_target = scatter([],[]);
        plot_target.Marker = 'x';
        plot_target.MarkerEdgeColor = 'r';
        axis([-1.5 1.5 -1.5 1.5]);
    
    %% ALGORITHM
    last_plot = tic; % Time of last plotting of laser data.
    hz = 3; % [Hz] Maximum Number of Times to Update LIDAR Display per Second.
    sct_overlay = 0; % Scatter Plot of Nearest Object for Overlay onto the LIDAR Data. 
    done = 0;
    while(~done)
        %rob.core.sendVelocity(0.045, 0.05);
        
        rs = rob.core.laser.LatestMessage.Ranges; % Store so all functions use same data set (lest there be an interim interrupt update)
        [l_near, i_near, th_near] = dist2nearestObject(rs);
        [x_near, y_near, ~] = RangeImage.irToXy(i_near, l_near);
        set(plot_target, 'XData', -y_near, 'YData', x_near);
        
        rob.trajectory_goTo(0.02, x_near, y_near, deg2rad(th_near));
        
        % Plot Laser Data
        if( (isempty(last_plot) || toc(last_plot)>1/hz) && laser_data_avail)
            last_plot = tic;
            
            fig = RangeImage.plot_rangeData(rob.core.laser.LatestMessage.Ranges, 1); % Plot Range Data
            %Plot nearest point on top of range data:
            figure(fig);
            set(0,'CurrentFigure',fig)
                if(sct_overlay == 0) %Overlay has not yet been performed.
                    hold on
                    sct_overlay =  scatter(-y_near,x_near, 'Marker', 'x', 'MarkerEdgeColor', 'r');
                    hold off
                else
                    set(sct_overlay, 'XData', -y_near, 'YData', x_near);
                end% sct_overlay==0
            
            laser_data_avail = 0; % Flag Laser Data as Processed.
            
        end  % last_time > 1/hz
            
        pause(0.05); % CPU Relief
    end % while ~done

    %% RESET ROBOTS & CLEAR MEMORY
    pause % Wait for instruction before closing

    rob.core.stopLaser();
    clear rob
    clear rasp
    clear robot
    close all
end % #Lab2_Solution_Connor

%% Distance to Nearest Object
% Returns distance, l,  to the nearest object with non-zero distance from the
% given range data, its index in the range data, idx, and its bearing, th.
function [l, idx, th] = dist2nearestObject(ranges)
    l = Inf;
    idx = 0;
    
    for i=(1:length(ranges))
        if(abs(ranges(i)) < abs(l) && ranges(i)~=0)
            l = ranges(i);
            idx = i;
        end % ranges<l
    end % for ranges
    
    th = RangeImage.index2bearing(idx);
end % 

% Callback Function on New Laser Range Data
%%DEPRECATED%%
function new_rangeData(~, event)
    global person laser_data_avail
    persistent last_time last_call person_initPos
    if isempty(last_call)
        last_call = tic; % time of last call to this function
    end
    
    laser_data_avail = 1; % Flag new Laser Data as Available
    
    % Move Person Up and Down from init a Maximum of dp Distance at speed vp
    dp = 1; vp = 0.06;
    if isempty(person_initPos); person_initPos = person.pose; end
    if (person.pose(2) > person_initPos(2)+dp || person.pose(2) < person_initPos(2)-dp)
        vp = -vp; % turn around if out of bounds
    end % if out of bounds
    person.pose = person.pose + [0 vp*toc(last_call) 0]; % Move
    
%     hz = 5; % [Hz] Maximum Number of Times to Update LIDAR Display per Second.
%     if(isempty(last_time) || toc(last_time)>1/hz )
%         last_time = tic;
%         plot_rangeData(event.Ranges);
%     end % last_time > 1/hz?
    last_call = tic;
end % #new_rangeData