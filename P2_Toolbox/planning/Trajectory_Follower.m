%% Trajectory Follower
% Class for Commanding a Given Robot to Follow a Given ReferenceTrajectory
classdef Trajectory_Follower < handle
    %% PROPERTIES
    properties(GetAccess=public, SetAccess=private)
        robot;              % Robot being Following the Given Trajectory
        rt;                 % ReferenceTrajectory Curve to Follow
        fbk_controller = FeedbackController.empty; % Feedback Controller
        
        u_comm_t = @(t)0;     % Function Handle for TrajectoryTime-Variant 
                            % Velocity Control Signal (Ffwd w/Fbk-trim)
        u_comm_s = @(s)0;     % Function Handle for TrajectoryTime-Variant 
                            % Velocity Control Signal (Ffwd w/Fbk-trim)
        
        send_delay = 0.164; % Delay in Command Sending to Robot
    end % Trajectory_Follower <- properties(public,private)
    
    properties(GetAccess=public, SetAccess=public)
        fbk_trim = 1;       % Include Feedback-Trim (if 0, only u_ffwd is used)
        
        u_ffwd_t= @(t)0;      % Function Handle for TrajectoryTime-Variant
                            % Feed-Forward Reference Velocity Curve
        u_fbk_t = @(t)0;      % Function Handle for TrajectoryTime-Variant 
                            % Feed-Back Velocity Responce (Feedback Trim)
        u_ffwd_s= @(s)0;      % Function Handle for TrajectoryDist-Variant
                            % Feed-Forward Reference Velocity Curve
        u_fbk_s = @(s)0;      % Function Handle for TrajectoryDist-Variant 
                            % Feed-Back Velocity Responce (Feedback Trim)
    end % Trajectory_Follower <- properties(public,public)

    %% METHODS
    methods
        %% Constructor
        function obj = Trajectory_Follower(rob, rt)
            obj.robot = rob;
            obj.rt = rt;
            obj.fbk_controller = FeedbackController(rob,rt);
            
            obj.u_ffwd_t = @(t) [rt.V_t(t) rt.om_t(t)];
            obj.u_fbk_t = @(t) obj.fbk_controller.getControl_t(t-obj.send_delay);
            %Feedforward w/Feedback-Trim:
            obj.u_comm_t = @(t)( obj.u_ffwd_t(t) + obj.u_fbk_t(t) );
            
            obj.u_ffwd_s = @(s) [rt.V_s(s) rt.om_s(s)];
            obj.u_fbk_s = @(s) obj.fbk_controller.getControl_s(s);
            %Feedforward w/Feedback-Trim:
            obj.u_comm_s = @(s)( obj.u_ffwd_s(s) + obj.u_fbk_s(s) );
        end % #Trajectory_Follower
        
        %% Follow Update
        % Commands the Robot to assume the Motion it should have at
        % Trajectory Time, t
        function follow_update_t(obj, t)
            if(obj.fbk_trim && t > 0.15*obj.rt.getFinalTime())
                u_t = obj.u_comm_t(t);
            else
                obj.u_fbk_t(t); % Just callin' it (so errors are still computed)
                u_t = obj.u_ffwd_t(t);
            end % fbk_trim?
            V = u_t(1);
            om = u_t(2);
%             if (V < 0)
%                 V = 0;
%                 om = 0;
%             end
            obj.robot.moveAt(V,om);
        end % #follow_update_t
        % Commands the Robot to assume the Motion it should have at
        % Trajectory Time, t
        function follow_update_s(obj, s)
            if(obj.fbk_trim)
                u_s = obj.u_comm_s(s);
            else
                obj.u_fbk_s(s); % Just callin' it (so errors are still computed)
                u_s = obj.u_ffwd_s(s);
            end % fbk_trim?
            V = u_s(1);
            om = u_s(2);
            obj.robot.moveAt(V,om);
        end % #follow_update_t
    end % Trajectory_Follower <- methods
end % Class Trajectory_Follower