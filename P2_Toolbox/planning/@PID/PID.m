classdef PID < handle
    
    %% properties
    properties (GetAccess = public, SetAccess = private)
        ttc;
        rob;
        
        %error values
        lastErrorX = 0;
        lastErrorY = 0;
        lastErrorTh = 0;
        errorIntegralX = 0;
        errorIntegralY = 0;
        errorIntegralTh = 0;
        
        %history of error readings stored as poses
        error_poses = [pose(0, 0, 0)];
        error_times;
    end %PID->properties(public, private)
    
    properties(GetAccess=public, SetAccess=public)
        % maximum values
        v_max = 0.2;
        w_max = 8;
        maxErrorIntegralX = 10;
        maxErrorIntegralY = 10;
        maxErrorIntegralTh = 10;
        
        %PID coefficients (1,0,0 -> off by default)
        k_p = 1;
        k_d = 0;
        k_i = 0;
        
        %correctiveTime (time in which errors should be corrected)
        correctiveTime = .3;
    end %PID -> properties(public, public)

    %% methods
    methods 
        
        %constructor
        function obj = PID(rob, ttc)
            obj.ttc = ttc;
            obj.rob = rob;
        end
        function [u_v, u_w] = getControl(obj, t)

            if isempty(obj.error_times) %initialize
                obj.error_times = [0];
            end % isEmpty(lastError)?
            t_last = obj.error_times(end);
            %% determine reference pose and estimated pose
            refPose = obj.ttc.getPose(t);
            curPose = obj.rob.hist_estPose(end);

            %% determine error values in every dimension
            %get most recent velocity readings
            V = obj.rob.hist_estVel(end).V;
            
            %error summing coefficients
            k_x = 1 / obj.correctiveTime;
            k_th = 1 / obj.correctiveTime;
            if V < 0.2 % V-floor for k_y to prevent it from becoming too large
                k_y = 0;
            else
                k_y = 2 / (abs(V) * obj.correctiveTime^2);
            end % k_y<0.2?
            
            wrp = (refPose.poseVec(1:2) - curPose.poseVec(1:2));
            
            thr = atan2(sin(curPose.th), cos(curPose.th));
            errorTh = refPose.th - curPose.th;
            errorTh = atan2(sin(errorTh),cos(errorTh));
            
            rrp = [cos(thr),-sin(thr); sin(thr), cos(thr)] \ wrp;
            
            errorX = rrp(1);
            errorY = rrp(2);
            errorTh = atan2(sin(errorTh), cos(errorTh)); % normalize angle
            dt = t - t_last;

            %% compute derivatives and integrals
            errorDerivativeX = (errorX-obj.lastErrorX)/ dt;
            errorDerivativeY = (errorY - obj.lastErrorY) / dt;
            errorDerivativeTh = (errorTh - obj.lastErrorTh) / dt;
            obj.errorIntegralX = obj.errorIntegralX + errorX*dt;
            if abs(obj.errorIntegralX) > obj.maxErrorIntegralX;
                sign = obj.errorIntegralX/abs(obj.errorIntegralX);
                obj.errorIntegralX= obj.maxErrorIntegralX * sign;
            end
            
            obj.errorIntegralY = obj.errorIntegralY + errorY*dt;
            if abs(obj.errorIntegralY) > obj.maxErrorIntegralY;
                sign = obj.errorIntegralY/abs(obj.errorIntegralY);
                obj.errorIntegralY= obj.maxErrorIntegralY * sign;
            end

            obj.errorIntegralTh = obj.errorIntegralTh + errorTh*dt;
            if abs(obj.errorIntegralTh) > obj.maxErrorIntegralTh;
                sign = obj.errorIntegralTh/abs(obj.errorIntegralTh);
                obj.errorIntegralTh= obj.maxErrorIntegralTh * sign;
            end
            
            ex = obj.k_p * errorX + obj.k_d * errorDerivativeX + ...
                obj.k_i * obj.errorIntegralX
            ey = obj.k_p * errorY + obj.k_d * errorDerivativeY + ...
                obj.k_i * obj.errorIntegralY
            eth = obj.k_p * errorTh + obj.k_d * errorDerivativeTh + ...
                obj.k_i * obj.errorIntegralTh
            %% compute actual control linear and rotational velocity
            u_v = ex * k_x;
            u_w = ey * k_y + eth * k_th;
%             u_v = errorX * k_x;
%             u_w = errorY * k_y + errorTh*k_th;

            %% ensure below ceiling
            if abs(u_v) > obj.v_max
                sign = u_v / abs(u_v);
                u_v = sign * obj.v_max;
            end

            if abs(u_w) > obj.w_max
                sign = u_w / abs(u_w);
                u_w = sign*obj.w_max;
            end
            
            % update history of error poses and times
            obj.error_poses(end+1) = pose(errorX, errorY, errorTh);
            obj.error_times(end+1) = t;

            % update 'last' variables
            obj.lastErrorX = errorX;
            obj.lastErrorY = errorY;
            obj.lastErrorTh = errorTh;
            end
        end
end