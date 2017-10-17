classdef Trajectory_CubicSpiral < ReferenceTrajectory
    %cubicSpiral Implements a planar trajectory specified in terms of three 
    % coefficients that adjust the terminal pose of theo robot. The initial
    % pose is assumed to be the origin with zero curvature. The terminal
    % curvature is forced to be zero.
    
    properties(Constant)

    end
    
    properties(Access = private)
        parms = [0 0 1];
        sgn=0.0;
        rampLength = 0.10;
    end
    
    properties(Access = public)
        numSamples = 0;
        distArray = [];
        timeArray = [];
        poseArray = [];
        curvArray = [];
        vlArray = []
        vrArray = []
        VArray = [];
        wArray = [];
        
        V_max = 0.2;% Maximum Wheel Velocity
    end
    
    % Inherited Abstract Methods (from ReferenceTrajectory)
    properties
        send_delay = 0;     % s, Delay from Command Send to When the Robot Begins Executing it.
        
        N_samples;      % Number of Samples in the Trajectory
    end % ReferenceTrajectory <-properties(Abstract)
    
    methods(Static = true)
        
        function makeLookupTable(scale)
            % Make 6 lookup tables to be used for cubicSpiral generation.
            % Principle is to sample densely in coefficient space and
            % integrate the curvature to find the curve. Then store the
            % result in a table that inverts the mapping.
            % On a 2.6 GHz core i7:
            % Scale = 100 takes 0.17 minutes
            % Scale = 50 takes 0.64 minutes
            % Scale = 10 takes 15 minutes
            % Scale = 2 takes 381 minutes
            % Scale = 1 takes > 125 hours (and still needs interpolation)
            
            startTic = tic();

            sMax = 1.0;   % length of curve
            qMax =  pi(); % max bearing
            qMin = -pi(); % min bearing
            tMax =  1.5*pi(); % max heading
            tMin = -1.5*pi(); % min heading

            numT = round(10*126/scale); % heading samples
            numQ = round(50*126/scale); % bearing samples

            % Do a quick performance estimate
            radius = 1.0;
            ds = radius*(qMax-qMin)/numQ;
            dt = (tMax-tMin)/numT;
            fprintf('Error predicted %f m in translation, %f rads in rotation\n',ds,dt);

            % Allocate tables
            a1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            b1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            r1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT); % energy integral

            a2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            b2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            r2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT); % energy integral

            clothSamples = 201;

            plotSamples = 10000;
            plotArrayX = zeros(plotSamples);
            plotArrayY = zeros(plotSamples);

            n = 1;
            aMax = 195.0/sMax/sMax;
            bMax = 440.0/sMax/sMax/sMax;

            % 15 minutes at scale = 10
            % X at scale = 1
            numA = 40000.0/scale;
            numB = 25000.0/scale;
            for a = -aMax:aMax/numA:aMax
                for b = -bMax:bMax/numB:bMax
                    ds = sMax/(clothSamples-1); 
                    x=0.0; y = 0.0; t = 0.0; r = 0.0;
                    broke = false;
                    k_s = 0.0;
                    s = 0.0;
                    for i=1:clothSamples
                        s = s + (ds /2.0);
                        % since s_f = 1
                        k_s = s * (a+b*s) * (s - 1.0);
                        t = t + k_s*(ds/2.0);
                        x = x + cos(t)*ds;
                        y = y + sin(t)*ds;
                        r = r + ((k_s^2) * ds);
                        s = s + (ds / 2.0);
                        t = t + k_s*(ds/2.0);
                        if ((t > tMax) || (t < tMin))
                            broke = true;
                            break; end
                    % Compute the curve. Break out of this loop, and then 
                    % immediately continue to next iteration of the for b loop 
                    % if tmax is exceeded in absolute value at any time.
                    end
                    if(broke == true); continue; end;

                    q = atan2(y,x);
                    %if(~aTab.isInBounds(q,t)); continue; end;
                    % This is faster
                    if(q<qMin || q>=qMax || t<tMin || t>=tMax) ; continue; end;
                    if(n <= plotSamples)
                        plotArrayX(n) = x;
                        plotArrayY(n) = y;
                    elseif(n == plotSamples+1)
                        if(scale > 20)
                            figure(1);
                            plot(plotArrayX(1:n-1),plotArrayY(1:n-1),'.k','MarkerSize',3);
                            hold on;
                            fprintf('Plot array dumping\n');
                        end
                        n = 0;
                        elapsedTime = toc(startTic);
                        fprintf('Took %f minutes\n',elapsedTime/60.0);
                    end;
                    n = n + 1;
                    % Store coesfficients.
                    r1Now = r1Tab.get(q,t);
                    r2Now = r2Tab.get(q,t);
                    if(abs(r1Now) > abs(r) && a >=0.0)
                        r1Tab.set(q,t,r);
                        a1Tab.set(q,t,a);
                        b1Tab.set(q,t,b);
                    elseif(abs(r2Now) > abs(r) && a <=0.0)
                        r2Tab.set(q,t,r);
                        a2Tab.set(q,t,a);
                        b2Tab.set(q,t,b);
                    end
                end

            end

            elapsedTime = toc(startTic);
            fprintf('Took %f minutes\n',elapsedTime/60.0);
            
            if(scale > 10)
                plot(plotArrayX(1:n-1),plotArrayY(1:n-1),'.k','MarkerSize',3);
            end
            
            save(strcat('cubicSpirals_Data_',num2str(scale)),'a1Tab','a2Tab','b1Tab','b2Tab','r1Tab','r2Tab');
            fileInfo = dir(strcat('cubicSpirals_Data_',num2str(scale),'.mat'));
            if(fileInfo.bytes > 95e6) % Split Up File if Too Large
                warning('Output File Too Large for Git, Splitting it Up...');
                Split_TCS_Data(scale);
                warning('Split Complete. Deleting Original...');
                delete(strcat('cubicSpirals_Data_',num2str(scale),'.mat'));
                warning('Origin Dumped.');
            end
            
            figure(2);
            I1 = mat2gray(a1Tab.cellArray, [-aMax aMax]);
            imshow(I1);
            xlabel('Heading'); ylabel('Bearing'); title('a1Tab');
            figure(3);
            I2 = mat2gray(a2Tab.cellArray, [-aMax aMax]);
            imshow(I2);
            xlabel('Heading'); ylabel('Bearing'); title('a2Tab');
            figure(4);
            I1 = mat2gray(b1Tab.cellArray, [-bMax bMax]);
            imshow(I1);
            xlabel('Heading'); ylabel('Bearing'); title('b1Tab');
            figure(5);
            I2 = mat2gray(b2Tab.cellArray, [-bMax bMax]);
            imshow(I2);
            xlabel('Heading'); ylabel('Bearing'); title('b2Tab');
        end
        
        % Plans a trajectory to Position (x,y,th) Relative to Current Robot
        % Position in the direction of sgn, with number of samples 
        % optionally specified by ns, and optionally loading data of a 
        % particular scale as specified by ds.
        function curve = planTrajectory(x,y,th,sgn, ns, ds)
            persistent data_loaded;
            persistent a1T a2T b1T b2T r1T r2T num_samp data_scale
            
            if(nargin > 4)
                num_samp = ns;
            elseif(isempty(num_samp))
                num_samp = 201; %default value
            end % nargin>4?
            
            if(nargin > 5)
                data_scale = ds;
                load_data(data_scale);
            elseif(isempty(data_scale))
                data_scale = 30; % default value
            end % nargin>5?
            
            if(isempty(data_loaded))
                load_data(data_scale);
            end
            
            function load_data(scale)
                try
                    load(strcat('cubicSpirals_Data_',num2str(scale)),'a1Tab','a2Tab','b1Tab','b2Tab','r1Tab','r2Tab');
                catch
                    load(strcat('cubicSpirals_Data_',num2str(scale),'_A'),'a1Tab','a2Tab');
                    load(strcat('cubicSpirals_Data_',num2str(scale),'_B'),'b1Tab','b2Tab');
                    load(strcat('cubicSpirals_Data_',num2str(scale),'_r'),'r1Tab','r2Tab');
                end
                data_loaded = true;
                a1T = a1Tab;a2T = a2Tab;b1T = b1Tab;b2T = b2Tab;r1T = r1Tab;r2T = r2Tab;
            end
            
            q = atan2(y,x);
            % set pi(0 equal to - pi() to avoid out of bounds in scalar
            % field.
            if(abs(q-pi()) < 1e-10) ; q = -pi(); end;
            t = th;
            a1 = a1T.get(q,t);
            a2 = a2T.get(q,t);
            b1 = b1T.get(q,t);
            b2 = b2T.get(q,t);
            r1 = r1T.get(q,t);
            r2 = r2T.get(q,t);

            oneBad = false;
            twoBad = false;
            if(isinf(a1) || isinf(b1))
                oneBad = true;
            end
            if(isinf(a2) || isinf(b2))
                twoBad = true;
            end;

            % Pick sole good one or the least curvy
            % I tried with and without this and it makes a huge difference! 
            % Its the secret to computing a table in reasonable time because the
            % least curvy is pretty accurate
            if(oneBad==true && twoBad==true)
                disp('No solution\n');
                curve = {};
                return;
            elseif(oneBad==true)
                au=a2;bu=b2;
            elseif(twoBad==true)
                au=a1;bu=b1;       
            elseif(r2 > r1)
                au=a1;bu=b1;  
            else
                au=a2;bu=b2;
            end

            % Plot the corresponding unit
            su = 1.0;
            clothu = Trajectory_CubicSpiral([au bu su],num_samp);

            %hold on;

            % Scale it up to the original
            xu = clothu.poseArray(1,clothu.numSamples);
            yu = clothu.poseArray(2,clothu.numSamples);
            Ru = sqrt(xu*xu+yu*yu);
            RO = sqrt(x*x+y*y);
            lam = RO/Ru;

            ss = lam;
            as = au/lam/lam/lam;
            bs = bu/lam/lam/lam/lam;
            
            if(sgn < 0)
                as = -as;  
                ss = -ss;
            end
            curve = Trajectory_CubicSpiral([as bs ss],num_samp);
        end
            
    end
    
    methods(Access = private)
        
       function setParms(obj,parms)
            obj.parms = parms;
        end
        
        function integrateCommands(obj)
            len = obj.numSamples;
            obj.distArray  = zeros(1,len);
            obj.poseArray  = zeros(3,len);
            obj.curvArray  = zeros(1,len);

            % Place robot in initial state
            obj.distArray(1) = 0.0;
            obj.poseArray(1,1) = 0.0;
            obj.poseArray(2,1) = 0.0;
            obj.poseArray(3,1) = 0.0;
            obj.curvArray(1) = 0.0;

            a = obj.parms(1);
            b = obj.parms(2);
            sf = obj.parms(3);
            ds = sf/(obj.numSamples-1);
            
            s = 0.0;
            xs = obj.getXVec();
            ys = obj.getYVec();
            ths = obj.getThVec();
            
            for i=1:obj.numSamples-1
                s = s + ds/2;
                obj.distArray(i+1) = s;
                obj.curvArray(i+1) = s*(a + b*s)*(s-sf);
                ths(i+1) = ths(i) + obj.curvArray(i)*(ds/2);
                xs(i+1) = xs(i) + cos(ths(i+1))*ds;
                ys(i+1) = ys(i) + sin(ths(i+1))*ds;
                ths(i+1) = ths(i+1) + obj.curvArray(i)*(ds/2);
                s = s + ds/2;
            end
            obj.poseArray(1,:) = xs;
            obj.poseArray(2,:) = ys;
            obj.poseArray(3,:) = ths;
       
            i = obj.numSamples;
            s = (i-1)*ds;  
            obj.distArray(i) = s;
            obj.curvArray(i) = 0.0;
        end   
        
        function computeTimeSeries(obj)
            % Find the times implied by the distances and the velocities
            len = obj.numSamples;
            obj.timeArray  = zeros(1,len);

            % Place robot in initial state
            obj.timeArray(1) = 0.0;

            for i=1:obj.numSamples-1
                ds = obj.distArray(i+1) - obj.distArray(i);
                V = obj.VArray(i);
                % Avoid division by zero
                if(abs(V) < 0.001); V = obj.VArray(i+1); end;

                obj.timeArray(i+1)= obj.timeArray(i)+ds/V;
            end
        end   
    end
            
    methods(Access = public)
        
        function obj = Trajectory_CubicSpiral(parms, numSamples, vm)
            % Constructs a cubicSpiral for the supplied parameters. The
            % parameters are in order [a b sf]. numSamples is the
            % number of integration steps used to integrate the entire 			
            % curve. The curve is integrated immediately when this
            % this constructor is called. Thereafter the public arrays 
            % distArray, poseArray, and curvArray are populated.
            % Once the curve is integrated, path velocities not exceeding
            % obj.V_max=vm and the time series relating to each path length
            % are computed.
            
            % Check if all Abstract Methods are Implemented:
            meta.abstractDetails(?Trajectory_CubicSpiral)
            
            if(nargin > 2)
                obj.V_max = vm;
            end
            if(nargin >= 2)
                obj.numSamples = numSamples;
                 obj.N_samples = numSamples;
                obj.parms = parms;
                
                obj.integrateCommands();
                obj.sgn = sign(parms(3)); % Critical for proper time series
                
                obj.planVelocities(obj.V_max);
            end
        end 
                
       function planVelocities(obj,Vmax)
            % Plan the highest possible velocity for the path where no
            % wheel may exceed Vmax in absolute value.
            obj.V_max = Vmax; % Update Class Property
            
            for i=1:obj.numSamples
                Vbase = Vmax;
                % Add velocity ramps for first and last 5 cm
                s = obj.distArray(i);
                sf = obj.distArray(obj.numSamples);
                if(abs(sf) > 2.0*obj.rampLength) % no ramp for short trajectories
                    sUp = abs(s);
                    sDn = abs(sf-s);
                    if(sUp < obj.rampLength) % ramp up
                        Vbase = Vbase * sUp/obj.rampLength;
                    elseif(sDn < 0.05) % ramp down
                        Vbase = Vbase * sDn/obj.rampLength;
                    end
                end
                % Now proceed with base velocity 
                %disp(Vbase);
                V = Vbase*obj.sgn; % Drive forward or backward as desired.
                K = obj.curvArray(i);
                w = K*V;
                vr = V + robotModel.W2*w;
                vl = V - robotModel.W2*w;               
                if(abs(vr) > Vbase)
                    vrNew = Vbase * sign(vr);
                    vl = vl * vrNew/vr;
                    vr = vrNew;
                end
                if(abs(vl) > Vbase)
                    vlNew = Vbase * sign(vl);
                    vr = vr * vlNew/vl;
                    vl = vlNew;
                end
                obj.vlArray(i) = vl;
                obj.vrArray(i) = vr;
                obj.VArray(i) = (vr + vl)/2.0;
                obj.wArray(i) = (vr - vl)/robotModel.W;
            end
            % Now compute the times that are implied by the velocities and
            % the distances.
            obj.computeTimeSeries();
        end
        
        function J = getJacobian(obj)
            eps = 0.00001;
            dp = [eps 0.0 0.0];
            clothPlus1 = Trajectory_CubicSpiral(obj.parms+dp,obj.numSamples);
            posePlus = clothPlus1.getFinalPose;
            poseObj = obj.getFinalPose();
            J(:,1) = (posePlus-poseObj)/eps;
           
            dp = [0.0 eps 0.0];
            clothPlus2 = Trajectory_CubicSpiral(obj.parms+dp,obj.numSamples);
            posePlus = clothPlus2.getFinalPose;
            poseObj = obj.getFinalPose();
            J(:,2) = (posePlus-poseObj)/eps;
            
            dp = [0.0 0.0 eps];
            clothPlus3 = Trajectory_CubicSpiral(obj.parms+dp,obj.numSamples);            
            posePlus = clothPlus3.getFinalPose;
            poseObj = obj.getFinalPose();
            J(:,3) = (posePlus-poseObj)/eps; 
        end
        
        function refineParameters(obj,goalPose)
            disp('Entering iterations');
            newParms = obj.getParms();
            newObj = Trajectory_CubicSpiral(newParms,5001); % hi samples for derivatives
            thisPose = newObj.getFinalPose();
            error = inf; n = 1;
            while(error > 0.001)
                %disp('desired pose');
                %disp(goalPose);
                J = newObj.getJacobian();
                if(rcond(J) < 1.0e-6)
                    fprintf('Unable to invert Jacobian. Exiting\n');
                    break;
                end
                delP = J^-1 * (goalPose-thisPose);
                newParms = newParms+delP';
                newObj.setParms(newParms);
                newObj.integrateCommands();
                %disp('this pose');
                thisPose = newObj.getFinalPose();
                error = norm(thisPose-goalPose);
                disp(error);
                if(n >=20)
                    fprintf('Trajectory_CubicSpiral failing to refine Parameters\n');
                    break;
                end
            end
        end
        
        function s  = getDistAtDist(obj,s)
            if( s < obj.distArray(1))
                s = 0.0;
            else
                s  = interp1(obj.distArray,obj.distArray,s,'pchip','extrap');  
            end
        end
        
        function K  = getCurvAtDist(obj,s)
            if( s < obj.distArray(1))
                K = 0.0;
            else
                K  = interp1(obj.distArray,obj.curvArray,s,'pchip','extrap');  
            end
        end
            
        function p  = getPoseAtDist(obj,s)
            x = interp1(obj.distArray,obj.poseArray(1,:),s,'pchip','extrap');
            y = interp1(obj.distArray,obj.poseArray(2,:),s,'pchip','extrap');
            th = interp1(obj.distArray,obj.poseArray(3,:),s,'pchip','extrap');
            p  = pose(x, y, th);  
        end
        
        function p  = getFinalPose(obj)
            p  = pose(obj.poseArray(:,obj.numSamples));  
        end
        
        function time  = getTrajectoryDuration(obj)
            time  = obj.timeArray(:,obj.numSamples);  
        end
        
        function dist  = getTrajectoryDistance(obj)
            dist  = obj.distArray(:,obj.numSamples);  
        end
        
        function V  = getVAtTime(obj,t)
            if( t < obj.timeArray(1))
                V = 0.0;
            elseif(t > obj.getFinalTime())
                V = obj.VArray(obj.numSamples);
            else
                V  = interp1(obj.timeArray,obj.VArray,t,'pchip','extrap');  
            end
        end
            
        function w  = getwAtTime(obj,t)
            if(t < obj.timeArray(1))
                w = 0.0;
            elseif(t > obj.getFinalTime())
                w = obj.wArray(obj.numSamples);
            else
                w  = interp1(obj.timeArray,obj.wArray,t,'pchip','extrap');  
            end
        end
            
        function p  = getPoseAtTime(obj,t)
            if(t > obj.getFinalTime())
                p = obj.getFinalPose();
            else
                x = interp1(obj.timeArray,obj.poseArray(1,:),t,'pchip','extrap');
                y = interp1(obj.timeArray,obj.poseArray(2,:),t,'pchip','extrap');
                th = interp1(obj.timeArray,obj.poseArray(3,:),t,'pchip','extrap');
                p  = pose(x,y,th);
            end
        end  
        
        function parms  = getParms(obj)
            parms  = obj.parms;  
        end
        
        function setSgn(obj,sgn)
            obj.sgn  = sgn;  
        end  
        
        function reflectX(obj)
            % reflect curve around X axis by modifying parameters
            obj.parms(1) = - obj.parms(1);  
            obj.parms(2) = - obj.parms(2);
            obj.integrateCommands();
        end 
        function reflectY(obj)
            % reflect curve around Y axis by modifying parameters
            obj.parms(2) = - obj.parms(2);  
            obj.parms(3) = - obj.parms(3); 
            obj.integrateCommands();
        end 
        
        function reflectXY(obj)
            % reflect curve around X and Y axes (i.e. through origin) by modifying parameters
            obj.parms(1) = - obj.parms(1);  
            obj.parms(3) = - obj.parms(3);
            obj.integrateCommands();
        end 
    end
    
    
    % Inherited Abstract Methods (from ReferenceTrajectory)
    methods
        
        % Compute Everything Necessary to Create a Follow-able Path
        function compute(obj)
            obj.integrateCommands();
            obj.planVelocities(obj.V_max);
        end
        
        function transMat = getTransformMat(obj)
            transMat = obj.init_pose.bToA()
        end
        % Transforms Every Pose in the Data-Set to World Coordinates based
        % on the Object's "init_pose" property.
        function offsetInitPose(obj)
            
            transformMat = obj.getTransformMat()
            offsetTh = obj.init_pose.th
            
            %iterate through all x, y, and th in poseArray and transform
            %   them
            s = 0.0
            xs = obj.getXVec()
            ys = obj.getYVec()
            ths = obj.getThVec()
            for i=1:obj.numSamples
                oldTh = ths(i);
                oldX = xs(i);
                oldY = ys(i);
                oldPose = [oldX; oldY; 1]
                newPose = transformMat * oldPose
                xs(i) = newPose(1);
                ys(i) = newPose(2);
                ths(i) = oldTh; %atan2(sin(oldTh + offsetTh), cos(oldTh + offsetTh));
            end
            obj.poseArray(1,:) = xs;
            obj.poseArray(2,:) = ys;
            obj.poseArray(3,:) = ths;
        end
        
        % Angular Velocity at Time:
        function om = getOmegaAtTime(obj,t)
            om = getwAtTime(obj,t);
        end
        % Path Curvature at Time:
        function K = getCurvAtTime(obj,t)
            K = obj.getCurvAtDist(obj.getSAtTime(t));
        end
        
        %For Inverting Parameterization if Necessary (Computationally
        %heavy)
        % Path Length at t:
        function s = getSAtTime(obj,t)
            if(t < obj.timeArray(1))
                s = 0.0;
            elseif(t > obj.getFinalTime())
                s = obj.distArray(obj.numSamples);
            else
                s  = interp1(obj.timeArray,obj.distArray,t,'pchip','extrap');  
            end
        end
        % Time at Path Length:
        function t = getTAtDist(obj,s)
            if(s < obj.distArray(1))
                t = 0.0;
            elseif (s > obj.getFinalDist())
                t = obj.getFinalTime();
            else
                t  = interp1(obj.distArray,obj.timeArray,s,'pchip','extrap');  
            end
        end
        
        % Angular Velocity at Path Length:
        function om = getOmegaAtDist(obj,s)
            if(s < obj.distArray(1))
                om = 0.0;
            else
                om  = interp1(obj.distArray,obj.wArray,s,'pchip','extrap');  
            end
        end
        % Velocity at Path Length:
        function V = getVAtDist(obj,s)
            if(s < obj.distArray(1))
                V = 0.0;
            else
                V  = interp1(obj.distArray,obj.VArray,s,'pchip','extrap');  
            end
        end
        
        
        %Return the Elapsed Time at the End of the Path:
        function tf = getFinalTime(obj)
            tf = obj.getTrajectoryDuration();
        end
        %Return the Path Length Covered at the End of the Path:
        function sf = getFinalDist(obj)
            sf = obj.getTrajectoryDistance();
        end
        %Returns the Velocity at the End of the Path:
        function vf = getFinalVelocity(obj); vf = obj.V_t(obj.t_f); end
        %Returns the Velocity at the End of the Path:
        function omf = getFinalOmega(obj); omf = obj.om_t(obj.t_f); end
        %Returns the Velocity at the End of the Path:
        function Kf = getFinalCurv(obj); Kf = obj.K_t(obj.t_f); end
        
        %Returns Vector of All X-Positions:
        function xs = getXVec(obj)
            xs = obj.poseArray(1,:);
        end
        %Returns Vector of All Y-Positions:
        function ys = getYVec(obj)
            ys = obj.poseArray(2,:);
        end
        %Returns Vector of All Headings:
        function ths = getThVec(obj)
            ths = obj.poseArray(3,:);
        end
        %Returns Vector of All Times:
        function ts = getTVec(obj)
            ts = obj.timeArray;
        end
        %Returns Vector of All Path Lengths:
        function ss = getSVec(obj)
            ss = obj.distArray;
        end
        %Returns Vector of All Poses:
        function ps = getPVec(obj)
            ps = obj.poseArray;
        end
        
    end % ReferenceTrajectory <-methods(Abstract)
end