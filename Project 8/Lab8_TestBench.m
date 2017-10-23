function Lab8_TestBench(robot_id)
global laser_plotting_data
    %% SETUP ROBOT
    rasp = raspbot(robot_id, [0; 0; pi/2]);
    rob = P2_Robot(rasp);
    
    %% SETUP MAPPING
    if(strcmp(robot_id,'sim'))
        bounds = 2*[0.5 0; 0.5 1; -0.5 1; -0.5 0]; % Inverted U-Shaped Container
        block = ShapeGen.rect(0.038,0.127);
        wm = WorldMap(rob, bounds);
            % Create a Ring of Blocks around Origin.
            r_min = 0.42;
            r_max = 1;
            n = 3;
            ths = (0 : pi/2/n : pi/2 + pi/2/n); % Go one block beyond first quadrant
            for th = ths
                obs = wm.addObstacle(block);
                r_rel = r_min + (r_max-r_min) * (th / (pi/2)); % Create Spiral
                obs.pose = [r_rel*cos(th) r_rel*sin(th) th];
            end
        wm.createMap();
    end
    
    %% INITIALIZE        
    laser_plotting_data = struct( ...
        'delay_width',0.1, ...  % Min amount of time /between/ lidar plots
        'last_time',0, ...      % Last time lidar data was plotted.
...%         'new_data',0, ...       % Flag for Availability of New Lidar Data 
        'raw_data',0, ...        % Last Captured Raw Lidar Data
        'colorize',0 ...         % Whether Plotted Data should be Colorized (by range)
    );

    fig = figure();
        grid on
        grid minor
    
        laser_plotting_data.colorize = 1;
        if laser_plotting_data.colorize
            plot_obj = scatter(0,0,36,0);
        else
            plot_obj = scatter(0,0,36);
        end
        set(gca, 'Xdir', 'reverse'); % Ensure Robot Y-Axis Points Left

        title({'LIDAR Data', '(Robot Reference Frame)'});
        xlabel('Y-Position [m]');
        ylabel('X-Position [m]');
        axis(1.2*[
            -RangeImage.MAX_RANGE RangeImage.MAX_RANGE ...
            -RangeImage.MAX_RANGE RangeImage.MAX_RANGE ...
        ]);
        
    rob.core.startLaser();
    rob.core.laser.NewMessageFcn = @processLaserData;
    
    pause(1); % Wait for system to enter steady-state
    
    clk = Clock();
    while(clk.time() < 30)
        V = 0; rho = 0.5;
        rob.moveAt(V,V/rho);
        
        %% Plot Laser Data
        % Update Lidar Plot Every "period" seconds (since /end/ of last plot).
        if( clk.time() - laser_plotting_data.last_time > laser_plotting_data.delay_width )
            r_img = RangeImage(laser_plotting_data.raw_data);
            figure(fig);
            r_img.plot(laser_plotting_data.colorize, plot_obj);
            
            r_img.findLineCandidates();
            r_img.plotLineCandidates();
            
        laser_plotting_data.last_time = clk.time();
        end
     pause(0.01); % CPU Relief
    end % while
    
end % #Lab8_TestBench

function processLaserData(~, evnt)
global laser_plotting_data
    laser_plotting_data.raw_data = evnt.Ranges;
end