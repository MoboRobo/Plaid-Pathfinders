%% Trajectory Follower
% Class for Commanding a Given Robot to Follow a Given Reference Trajectory
classdef Trajectory_Follower < handle
    %% PROPERTIES
    properties(GetAccess=public, SetAccess=private)
        robot;              % Robot being Following the Given Trajectory
        ttc;                % Trajectory Time Curve to Follow
        pid_controller;     % PID Controller
        
        u_comm = @(t)0;     % Function Handle for TrajectoryTime-Variant 
                            % Velocity Control Signal (Ffwd w/Fbk-trim)
    end % Trajectory_Follower <- properties(public,private)
    
    properties(GetAccess=public, SetAccess=public)
        u_ffwd= @(t)0;      % Function Handle for TrajectoryTime-Variant
                            % Feed-Forward Reference Velocity Curve
        u_fbk = @(t)0;      % Function Handle for TrajectoryTime-Variant 
                            % Feed-Back Velocity Responce (Feedback Trim)
    end % Trajectory_Follower <- properties(public,public)

    %% METHODS
    methods
        %% Constructor
        function obj = Trajectory_Follower(rob, ttc)
            obj.robot = rob;
            obj.ttc = ttc;
            obj.pid_controller = PID(rob,ttc);
            
            obj.u_ffwd = @(t) [ttc.getV(t) ttc.getOmega(t)];
%             obj.u_fbk = @(t) [0 0];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TODO: Change This.
            obj.u_fbk = @(t) obj.pid_controller.getControl(t);
            %Feedforward w/Feedback-Trim:
            obj.u_comm = @(t)( obj.u_ffwd(t) + obj.u_fbk(t) );
        end % #Trajectory_Follower
        
        %% Follow Update
        % Commands the Robot to assume the Motion it should have at
        % Trajectory Time, t
        function follow_update(obj, t)
            u = obj.u_comm(t);
            V = u(1);
            om = u(2);
            obj.robot.moveAt(V,om);
        end % #follow_update
    end % Trajectory_Follower <- methods
end % Class Trajectory_Follower