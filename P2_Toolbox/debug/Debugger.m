% Class Managing Robot Debugging 
classdef Debugger
    %% PROPERTIES
    properties (GetAccess=public, SetAccess=private)
        ON = 0;      % Is Debugging On or Off. Default off. (1->ON, 0->OFF)
        robot;       % Robot this Debugger is Tracking
        on_time;     % Time (tic) that the debugger started
    end % Debugger->properties(public,private)
    
    methods
        %% Constructor
        % r - Robot this Debugger is Tracking
        function obj = Debugger(r)
            if nargin>0
                if isa(r,'P2_Robot')
                    obj.robot = r;
                    
                    %Capture initial state of robot:
                    obj.resetData();
                else
                    error('Debugger Robot must be a P2_Ronot')
                end % r is P2_Robot?
            else
                error('Must give Debugger a Robot to track')
            end% nargin>0?
        end % #Debugger Constructor
        
        %% ON
        %Turn Debugger On
        function turnOn(obj)
            obj.ON = 1;
        end
        
        %% OFF
        %Turn Debugger Off
        function turnOff(obj)
            obj.ON = 0;
        end
        
        %% Reset Data
        % Resets Robot Data (time, etc) that the Debugger Stores.
        function resetData(obj)
            obj.on_time = tic;
        end % #resetData
        
        %% Echo
        % Echoes the Input i to the Command line if Debugging is On
        function i = echo(obj, i)
            if(obj.ON)
                i
            end
        end % #echo
        
        %% Plot Encoder Position
        %Continuously Plot the Average Encoder Position (first call to this
        %function automatically creates figure with plots for average,
        %left, and right encoder values). Everytime this function is
        %called, the plot is updated.
        % obj - debugger object
        % MAX_VAL - Maximum Sensor Value
        function plot_encoderPosition(obj, MAX_VAL)
            persistent fig ...      % Figure containing plot
                       pl_avg ...   % Plot of Average Encoder Readings
                       pl_left ...  % Plot of Left Encoder Readings
                       pl_right ... % Plot of Right Encoder Readings
                       ts ...       % Vector of Plot Call Times (real-time)
                       ds ls rs     % Vector of Avg,Left,Right Encoder Values at Each Call
                   
            if isempty(fig) %Initialize Plots and Data (only once on first call)
                % Populate with initial data:
                ts = [obj.robot.tripTime()]; % Time Data
                ds = [obj.robot.avg_tripDist()]; % Distance Data
                ls = [obj.robot.left_tripDist()];
                rs = [obj.robot.right_tripDist()];

                fig = figure('Name', 'Encoder Readings (debugging)');
                hold on
                pl_avg = plot(ds, '-k');
                pl_left = plot(ls, 'b');
                pl_right = plot(rs, 'r');
                    ylim([0 MAX_VAL]) % <-MAX Y-VALUE
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

            if obj.ON
                %Update Data
                ts(end+1) = obj.robot.tripTime(); % Time Data
                ds(end+1) = obj.robot.avg_tripDist(); % Distance Data
                ls(end+1) = obj.robot.left_tripDist();
                rs(end+1) = obj.robot.right_tripDist();
                
                %Plot
                hold on
                    set(pl_avg, 'XData',ts, 'YData',ds)
                    set(pl_left, 'XData',ts, 'YData',ls)
                    set(pl_right, 'XData',ts, 'YData',rs)
                hold off
                refreshdata
                drawnow
            end % DEBUG.ON?
        end % #plot_EncoderPosition
        
    end % Debugger->methods
    
end % Debugger