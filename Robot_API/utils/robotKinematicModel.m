classdef robotKinematicModel < handle
    %robotKinematicModel A convenience class for storing robot physical 
    % and performing related kinematic transforms. You can reference the
    % defined constants via the clas name with robotKinematicModel.W2 for
    % example because they are constant properties and therefore associated
    % with the class rather than any instance. Similiarly, the kinematics
    % routines are referenced from the class name as well.
    
    properties(Constant)
        %W  = 9.25*2.54/100;   % NEATO wheel tread in m
        %W2 = 9.25*2.54/2/100; % NEATO 1/2 wheel tread in m
        W  = 0.08675;              % Raspbot wheel tread in m
        W2 = 0.08675/2.0;             % Raspbot 1/2 wheel tread in m
        maxWheelVelocity = 0.5 % max of either wheel in m/sec
        
        %rad = .165;             % NEATO robot body radius id 12.75/2 inches
        rad = 0.06;
        %frontOffset = 6*0.0254; % NEATO front surface is 6 inch fwd of axle center
        frontOffset = 2.625*0.0254; % front robot surface is 2-5/8 inch fwd of axle center
        objOffset = 0.015;      % offset from sail face to front of sail
		%laser_l = -0.100;      % Neato laser offset (laser is behind wheels)
        laser_l = -0.000;       % Raspbot laser offset
		laser_rad = 0.04;       % laser housing radius
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
    end
    
    methods(Static = true)
        
        function [V, w] = vlvrToVw(vl, vr)
        % Converts wheel speeds to body linear and angular velocity.
            V = (vr + vl)/2.0;
            w = (vr - vl)/robotKinematicModel.W;
        end
        
        function [vl, vr] = VwTovlvr(V, w)
        % Converts body linear and angular velocity to wheel speeds.
            vr = V + robotKinematicModel.W*w;
            vl = V - robotKinematicModel.W*w;
        end
        
        function bodyPts = bodyGraph()
            % return an array of points that can be used to plot the robot
            % body in a window.
            % angle arrays
            step = pi/20;
            cir = 0: step: 2.0*pi;

            % circle for laser
            lx = robotKinematicModel.laser_rad*cos(cir);
            ly = robotKinematicModel.laser_rad*sin(cir);

            % body with implicit line to laser circle
            bx = [robotKinematicModel.rad*cos(cir) lx];
            by = [robotKinematicModel.rad*sin(cir) ly];
            
            %create homogeneous points
            bodyPts = [bx ; by ; ones(1,size(bx,2))];
        end        
    end
    
    methods(Access = private)
        
    end
            
    methods(Access = public)
        

    end
end