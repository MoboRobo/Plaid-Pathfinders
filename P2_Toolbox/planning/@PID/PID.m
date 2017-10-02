classdef PID < handle
    
    %% properties
    properties (GetAccess = public, SetAccess = private)
        ttc;
        rob;
        
        %error values
        lastErrorVel = 0;
        lastErrorOm = 0;
        errorIntegralVel = 0;
        errorIntegralOm = 0;
        
        %history of error readings stored as poses
        error_poses = [pose(0, 0, 0)];
        error_times;
    end %PID->properties(public, private)
    
    properties(GetAccess=public, SetAccess=public)
        % maximum values
        v_max = 0.2;
        w_max = 2*pi;
        maxErrorIntegralVel = 10;
        maxErrorIntegralOm = 10;
        
        %PID coefficients
        k_p = 3;
        k_d = 0.1;
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
        function [u_v u_w] = getControl(obj, t)

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
            k_y = 2 / (abs(V) * obj.correctiveTime^2);
            wrp = (refPose.poseVec(1:2) - curPose.poseVec(1:2));
            thr = curPose.th;
            errorTh = atan2(sin(refPose.th), cos(refPose.th)) - ...
                atan2(sin(curPose.th), cos(curPose.th));
            rrp = inv([cos(thr), -sin(thr); sin(thr), cos(thr)]) * wrp
            errorX = rrp(1);
            errorY = rrp(2);
            errorTh = atan2(sin(errorTh), cos(errorTh)); % normalize angle
            errorVel = k_x * errorX;
            errorOm = k_y * errorY + k_th * errorTh;
            dt = t - t_last;
            tt.hist_encError.x = errorX;

            %% compute derivatives and integrals
            errorDerivativeVel = (errorVel-obj.lastErrorVel)/ dt;
            errorDerivativeOm = (errorOm - obj.lastErrorOm) / dt;
            obj.errorIntegralOm = obj.errorIntegralOm + errorOm*dt;
            if abs(obj.errorIntegralOm) > obj.maxErrorIntegralOm;
                sign = obj.errorIntegralOm/abs(obj.errorIntegralOm);
                obj.errorIntegralOm= obj.maxErrorIntegralOm * sign;
            end
            
            obj.errorIntegralVel = obj.errorIntegralVel + errorVel*dt;
            if abs(obj.errorIntegralVel) > obj.maxErrorIntegralVel;
                sign = obj.errorIntegralVel/abs(obj.errorIntegralVel);
                obj.errorIntegralVel= obj.maxErrorIntegralVel * sign;
            end


            %% compute actual control linear and rotational velocity
            u_v = obj.k_p * errorVel + obj.k_d * errorDerivativeVel + ...
                obj.k_i*obj.errorIntegralVel;
            u_w = obj.k_p * errorOm + obj.k_d * errorDerivativeOm + ...
                obj.k_i * obj.errorIntegralOm;
            u_v = errorVel;
            u_w = errorOm;
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
            obj.lastErrorVel = errorVel;
            obj.lastErrorOm = errorOm;
            end
        end
end