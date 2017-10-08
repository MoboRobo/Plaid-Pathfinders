%% Trajectory Follower
% Class for Commanding a Given Robot to Follow a Given ReferenceTrajectory
classdef Trajectory_Follower < handle
    %% PROPERTIES
    properties(GetAccess=public, SetAccess=private)
        robot;              % Robot being Following the Given Trajectory
        rt;                 % ReferenceTrajectory Curve to Follow
        pid_controller;     % PID Controller
        
        u_comm_t = @(t)0;     % Function Handle for TrajectoryTime-Variant 
                            % Velocity Control Signal (Ffwd w/Fbk-trim)
    end % Trajectory_Follower <- properties(public,private)
    
    properties(GetAccess=public, SetAccess=public)
        fbk_trim = 1;       % Include Feedback-Trim (if 0, only u_ffwd is used)
        
        u_ffwd_t= @(t)0;      % Function Handle for TrajectoryTime-Variant
                            % Feed-Forward Reference Velocity Curve
        u_fbk_t = @(t)0;      % Function Handle for TrajectoryTime-Variant 
                            % Feed-Back Velocity Responce (Feedback Trim)
    end % Trajectory_Follower <- properties(public,public)

    %% METHODS
    methods
        %% Constructor
        function obj = Trajectory_Follower(rob, rt)
            obj.robot = rob;
            obj.rt = rt;
            obj.pid_controller = PID(rob,rt);
            
            obj.u_ffwd_t = @(t) [rt.V_t(t) rt.om_t(t)];
            obj.u_fbk_t = @(t) obj.pid_controller.getControl_t(t);
            %Feedforward w/Feedback-Trim:
            obj.u_comm_t = @(t)( obj.u_ffwd_t(t) + obj.u_fbk_t(t) );
        end % #Trajectory_Follower
        
        %% Follow Update
        % Commands the Robot to assume the Motion it should have at
        % Trajectory Time, t
        function follow_update_t(obj, t)
            if(obj.fbk_trim)
                u_t = obj.u_comm_t(t);
            else
                obj.u_fbk_t(t); % Just callin' it (so errors are still computed)
                u_t = obj.u_ffwd_t(t);
            end % fbk_trim?
            V = u_t(1);
            om = u_t(2);
            obj.robot.moveAt(V,om);
        end % #follow_update_t
    end % Trajectory_Follower <- methods
end % Class Trajectory_Follower