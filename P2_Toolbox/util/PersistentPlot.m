%% PersistentPlot
% Small wrapper class for a plot which sticks around and can have its x
% and y values updated over time.
classdef PersistentPlot < handle
    properties(GetAccess = public, SetAccess = private)
        fig;        % Figure that this plot is assigned to
        core;       % Core Matlab plot object
    end % properties
    
    methods
        %% Constructor
        % f - figure this plot belongs to
        % xs - initial x data
        % ys - initial y data
        function obj = PersistentPlot(f, xs, ys)
            obj.fig = figure(f);
            hold on
                obj.core = plot(xs,ys);
            hold off
        end % #PersistentPlot
        
        %% Add X
        % Adds an entry, x, to the X Data
        function addX(obj, x)
            set(obj.core, 'XData', [get(obj.core, 'XData') x]);
        end % #addX
        %% Add Y
        % Adds an entry, y, to the Y Data
        function addY(obj, y)
            set(obj.core, 'YData', [get(obj.core, 'YData') y]);
        end % #addY
        %% Add XY
        % Adds an entry, x, to the X Data and an entry, y, to the Y Data
        function addXY(obj, x, y)
            set(obj.core, 'XData', [get(obj.core, 'XData') x], ...
                'YData', [get(obj.core, 'YData') y]);
        end % #addXY
        
        %% Reset X
        % Resets the X Data to the vector xs
        function resetX(obj, xs)
            set(obj.core, 'XData', xs);
        end % #resetX
        %% Reset Y
        % Resets the Y Data to the vector ys
        function resetY(obj, ys)
            set(obj.core, 'YData', ys);
        end % #resetY
        %% Reset XY
        % Resets the X Data to the vector xs and Y Data to the vector ys
        function resetXY(obj, xs, ys)
            set(obj.core, 'XData', xs, 'YData', ys);
        end % #resetY
    end
end % class PersistentPlot