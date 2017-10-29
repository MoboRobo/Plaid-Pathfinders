classdef FeedbackController < handle
    
    %% properties
    properties (GetAccess = public, SetAccess = private)
        rob;
        
        %error values
        lastErrorX = 0;
        lastErrorY = 0;
        lastErrorTh = 0;
        errorIntegralX = 0;
        errorIntegralY = 0;
        errorIntegralTh = 0;
        
        lastErrorX_s = 0; % w.r.t. s
        lastErrorY_s = 0;
        lastErrorTh_s = 0;
        errorIntegralX_s = 0;
        errorIntegralY_s = 0;
        errorIntegralTh_s = 0;
        
        %history of error readings stored as poses
        error_poses = slidingFifo(10000, pose(0, 0, 0));
        error_times = slidingFifo(10000, 0);
        error_dists = slidingFifo(10000, 0);
        %commanded linear velocities parameterized by time
        comm_V_t = slidingFifo(10000, struct('comm_v', 0, 't', 0));
        %commanded rotational velocities parameterized by time
        comm_W_t = slidingFifo(10000, struct('comm_w', 0, 't', 0));
        comm_V_s; %commanded linear velocities parameterized by arc length
        comm_W_s; %commanded rotational velocities parameterized by arc length
    end %FeedbackController<-properties(public, private)
    
    properties(GetAccess=public, SetAccess=public)
        rt; % Reference Trajectory
        
        % maximum values
        v_max = 0.2;
        w_max = 8;
        maxErrorIntegralX = 10;
        maxErrorIntegralY = 10;
        maxErrorIntegralTh = 10;
                
        % Proportional Control:
        %correctiveTime (time in which errors should be corrected)
        correctiveTime = .3;
        
        % Whether this is controlling a pure turn (no k_x,k_y):
        isPureTurn = 0;
        
        % PID Layer:
        %PID coefficients (0,0,0 -> off by default)
        k_p = 0;
        k_d = 0;
        k_i = 0;

    end %FeedbackController <- properties(public, public)

    %% methods
    methods 
        
        %constructor
        function obj = FeedbackController(rob, rt)
            obj.rt = rt;
            obj.rob = rob;
        end
        function [u_v, u_w] = getControl_t(obj, t)
            
            t_last = obj.error_times.last();
            %% determine reference pose and estimated pose
            refPose = obj.rt.getPoseAtTime(t);
            curPose = obj.rob.measTraj.p_f;

            %% determine error values in every dimension
            %get most recent velocity readings
            V = obj.rob.measTraj.V_f;
            
            %error summing coefficients
            k_x = 1 / obj.correctiveTime;
            k_th = 1 / obj.correctiveTime;
            if V < 0.03 % V-floor for k_y to prevent it from becoming too large
                k_y = 0;
%                 warning('Low Velocity');
            else
                k_y = 2 / (abs(V) * obj.correctiveTime^2);
            end % k_y<0.03?
            
            wrp = (refPose.poseVec(1:2) - curPose.poseVec(1:2));
            
            thr = atan2(sin(curPose.th), cos(curPose.th));
            errorTh = atan2(sin(refPose.th),cos(refPose.th)) ...
                    - atan2(sin(curPose.th),cos(curPose.th));
            errorTh = atan2(sin(errorTh),cos(errorTh));
            
            rrp = [cos(thr),-sin(thr); sin(thr), cos(thr)] \ wrp;
            
            errorX = rrp(1);
            errorY = rrp(2);
            errorTh = atan2(sin(errorTh), cos(errorTh)); % normalize angle
            dt = t - t_last;

            %% compute derivatives and integrals            
            if(dt <= 0)
                errorDerivativeX = 0;
                errorDerivativeY = 0;
                errorDerivativeTh = 0;
            
            else
                errorDerivativeX = (errorX-obj.lastErrorX)/ dt;
                errorDerivativeY = (errorY - obj.lastErrorY) / dt;
                errorDerivativeTh = (errorTh - obj.lastErrorTh) / dt;
            end
            obj.errorIntegralX = obj.errorIntegralX + errorX*dt;
            if abs(obj.errorIntegralX) > obj.maxErrorIntegralX
                sign = obj.errorIntegralX/abs(obj.errorIntegralX);
                obj.errorIntegralX= obj.maxErrorIntegralX * sign;
            end
            
            obj.errorIntegralY = obj.errorIntegralY + errorY*dt;
            if abs(obj.errorIntegralY) > obj.maxErrorIntegralY
                sign = obj.errorIntegralY/abs(obj.errorIntegralY);
                obj.errorIntegralY= obj.maxErrorIntegralY * sign;
            end

            obj.errorIntegralTh = obj.errorIntegralTh + errorTh*dt;
            if abs(obj.errorIntegralTh) > obj.maxErrorIntegralTh
                sign = obj.errorIntegralTh/abs(obj.errorIntegralTh);
                obj.errorIntegralTh= obj.maxErrorIntegralTh * sign;
            end
            
            ex =obj.k_p * errorX + obj.k_d * errorDerivativeX + ...
                obj.k_i * obj.errorIntegralX;
            ey = obj.k_p * errorY + obj.k_d * errorDerivativeY + ...
                obj.k_i * obj.errorIntegralY;
            eth = obj.k_p * errorTh + obj.k_d * errorDerivativeTh + ...
                obj.k_i * obj.errorIntegralTh;
            % compute actual control linear and rotational velocity
            if(obj.isPureTurn)
                u_v = 0;
                u_w = errorTh*k_th + eth;
                warning('Is Pure Turn');
            else
                u_v = errorX*k_x + norm([ex ey]);
                u_w = errorY*k_y + errorTh*k_th + eth;
            end
%             u_v = errorX * k_x;
%             u_w = errorY * k_y + errorTh*k_th;
            
            % update history of error poses and times
            obj.error_poses.add(pose(errorX, errorY, errorTh));
            obj.error_times.add(t);
            
            %update comm_v_t and comm_W_t
            
            obj.comm_V_t.add(struct('comm_v', u_v, 't', t));
            obj.comm_W_t.add(struct('comm_w', u_w, 't', t));
            % update 'last' variables
            obj.lastErrorX = errorX;
            obj.lastErrorY = errorY;
            obj.lastErrorTh = errorTh;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [u_v, u_w] = getControl_s(obj, s)

            s_last = obj.error_dists(end);
            %% determine reference pose and estimated pose
            refPose = obj.rt.getPoseAtDist(s);
            curPose = obj.rob.measTraj.p_f;

            %% determine error values in every dimension
            %get most recent velocity readings
            V = obj.rob.measTraj.V_f;
            
            %error summing coefficients
            k_x = 1 / obj.correctiveTime;
            k_th = 1 / obj.correctiveTime;
            if V < 0.03 % V-floor for k_y to prevent it from becoming too large
                k_y = 0;
                warning('Low Velocity');
            else
                k_y = 2 / (abs(V) * obj.correctiveTime^2);
            end % k_y<0.03?
            
            wrp = (refPose.poseVec(1:2) - curPose.poseVec(1:2));
            
            thr = atan2(sin(curPose.th), cos(curPose.th));
            errorTh_s = refPose.th - curPose.th;
            errorTh_s = atan2(sin(errorTh_s),cos(errorTh_s));
            
            rrp = [cos(thr),-sin(thr); sin(thr), cos(thr)] \ wrp;
            
            errorX_s = rrp(1);
            errorY_s = rrp(2);
            errorTh_s = atan2(sin(errorTh_s), cos(errorTh_s)); % normalize angle
            ds = s - s_last;

            %% compute derivatives and integrals            
            if(ds == 0)
                errorDerivativeX_s = 0;
                errorDerivativeY_s = 0;
                errorDerivativeTh_s = 0;
            
            else
                errorDerivativeX_s = (errorX_s - obj.lastErrorX_s)/ ds;
                errorDerivativeY_s = (errorY_s - obj.lastErrorY_s) / ds;
                errorDerivativeTh_s = (errorTh_s - obj.lastErrorTh_s) / ds;
            end
            obj.errorIntegralX_s = obj.errorIntegralX_s + errorX_s*ds;
            if abs(obj.errorIntegralX_s) > obj.maxErrorIntegralX
                sign = obj.errorIntegralX_s/abs(obj.errorIntegralX_s);
                obj.errorIntegralX_s = obj.maxErrorIntegralX_s * sign;
            end
            
            obj.errorIntegralY_s = obj.errorIntegralY_s + errorY_s*ds;
            if abs(obj.errorIntegralY_s) > obj.maxErrorIntegralY
                sign = obj.errorIntegralY_s/abs(obj.errorIntegralY_s);
                obj.errorIntegralY_s = obj.maxErrorIntegralY * sign;
            end

            obj.errorIntegralTh_s = obj.errorIntegralTh_s + errorTh_s*ds;
            if abs(obj.errorIntegralTh_s) > obj.maxErrorIntegralTh
                sign = obj.errorIntegralTh_s/abs(obj.errorIntegralTh_s);
                obj.errorIntegralTh_s= obj.maxErrorIntegralTh * sign;
            end
            
            ex =obj.k_p * errorX_s + obj.k_d * errorDerivativeX_s + ...
                obj.k_i * obj.errorIntegralX_s;
            ey = obj.k_p * errorY_s + obj.k_d * errorDerivativeY_s + ...
                obj.k_i * obj.errorIntegralY_s;
            eth = obj.k_p * errorTh_s + obj.k_d * errorDerivativeTh_s + ...
                obj.k_i * obj.errorIntegralTh_s;
            % compute actual control linear and rotational velocity
            if(obj.isPureTurn)
                u_v = 0;
                u_w = errorTh_s*k_th + eth;
            else
                u_v = errorX_s*k_x + norm([ex ey]);
                u_w = errorY_s*k_y + errorTh_s*k_th + eth;
            end
%             u_v = errorX_s * k_x;
%             u_w = errorY_s * k_y + errorTh_s*k_th;

            %% ensure below ceiling
            if abs(u_v) > obj.v_max
                fprintf('v greater than v_max!');
                sign = u_v / abs(u_v);
                u_v = sign * obj.v_max;
            end

            if abs(u_w) > obj.w_max
                fprintf('omega greater than omega_max');
                sign = u_w / abs(u_w);
                u_w = sign*obj.w_max;
            end
            
            % update history of error poses and times
            obj.error_poses.add(pose(errorX, errorY, errorTh));
            obj.error_dists.add(s);
            % update 'last' variables
            obj.lastErrorX_s = errorX_s;
            obj.lastErrorY_s = errorY_s;
            obj.lastErrorTh_s = errorTh_s;
        end
    end
end
       