classdef raspbot < handle
    
    properties (GetAccess='public', SetAccess='private')
		name = 'sim';
		ws = 0;
		vel_pub;
		laser_pub;
		fork_pub;		
		kill_pub;
		encoders;
		laser;
		sim_robot;
		map;
        camera;
        speech_pub;
%        fileWrite_pub;
        playFile_pub;
    end
    
    properties (Hidden, GetAccess='private', SetAccess='private')
		sim = true;
		update_timer;
		stamper;
		last_update;
        vel_timer;
        last_vel_time = 0;

		ph = [-1 -1]; %plot handle
		ah;
		dist_since_cmd = 0;
		last_x = 0;
		last_y = 0;
		last_th = 0;
		
		enc_cnt = 0;
		laser_cnt = 0;
		plot_cnt = 5;
		last_l = 0;
		last_r = 0;
		vel_cmd = [0 0];
		
		laser_beam_pos = 0;
		sim_laser_on = false;
		has_map = false;
        
        fifoMutex = 0;
        inited=0;
    end
    
    properties (Constant, Hidden, GetAccess='private')
		max_vel = 0.5;
        min_vel_period = 0.1;
        default_gateway = '192.168.0.1';
        retries = 5;
		
		sim_freq = 90;	
		%Need to update to 5?	
		% enc_prescaler = 5; % 50Hz enc updates	
		enc_prescaler = 5; % 30 Hz enc updates
		
        laser_prescaler = 22; % ~4 Hz raycasting, full sweep
		laser_sweep = 360;
		laser_range = 4.5;
		
		plot_prescaler = 5; %90/5 = 18 Hz
		
		% not sure if enc_delay has changed.
		enc_delay = .25; %enc and vel delay
		laser_delay = .4; 
		max_dist = .5;

		%Variable to scale velocities down.
		limit_wheel_speeds = 1;      
    end
    
    methods
		function r = raspbot(name,pose)
            
            if(nargin == 0)
                r.name = 'RaspBot';
            elseif(nargin > 0)
				r.name = name;
            end
            
            % Addpath to files.
            fname = which('raspbot.m');
            fname = strrep(fname,'raspbot.m','');
            addpath(genpath(fname));

            if(nargin > 1)
                if( all( size(pose) == [3 1] ) )
                    initialPose = pose;
                else
                    error('Initial pose must be 3 x 1');
                end
            else
                initialPose = [0;0;0];
            end
            if(nargin > 0 && (strcmp(name,'sim') || strcmp(name,'manual_sim') ) )
                r.sim_robot =  simRobot(r.enc_delay, initialPose, 0, 0, false);
                % the fireup is done later too now, when first command goes out.
                % this one is to fill up the encoders structure
                r.sim_robot.fireUpForRealTime();
                r.inited = 0;
				
				r.stamper = tic();
				r.last_update = tic();
				
% 				r.encoders = SimSubscriber(struct('left',0,'right',0));
				r.encoders = SimSubscriber(struct('Vector',struct('X',0,'Y',0)));
                r.laser = SimSubscriber(struct('Ranges',zeros(1,360)));
				
				if(~strcmp(name,'manual_sim'))
					r.update_timer = timer;
                    r.update_timer.Tag = 'raspbot_sim';
					r.update_timer.ExecutionMode = 'fixedRate';
					r.update_timer.BusyMode = 'queue';
                    %r.update_timer.BusyMode = 'drop';
					r.update_timer.TimerFcn = @r.simUpdate;
                    s = warning('off', 'MATLAB:TIMER:RATEPRECISION');
					r.update_timer.Period = 1/r.sim_freq;
                    warning(s);
				
					start(r.update_timer);
				end
            else
                r.sim = 0;
                iters = 0;
                while r.ws == 0 && iters < r.retries
                    try
                        rosinit('http://192.168.0.1:11311')
                        r.ws = 1;
                    catch e
                        r.ws = 0;
                        rosshutdown;
                    end
                    iters = iters + 1;
                end
                
                if r.ws == 0
                    throw(MException('RASPBOT:NoNetworkConnection', 'ROS Master not reachable'));
                end
                
                r.vel_timer = tic();
                r.last_vel_time = toc(r.vel_timer);
                r.vel_pub=rospublisher('/cmd_vel', rostype.geometry_msgs_Twist);
                r.laser_pub=rospublisher('/laser',rostype.std_msgs_Int8);
                r.fork_pub=rospublisher('/forks',rostype.std_msgs_Int8);
                r.encoders=rossubscriber('/enc');
                r.laser=rossubscriber('/scan');
                r.camera=rossubscriber('/camera/image/compressed');
                r.speech_pub=rospublisher('/textToSpeech', rostype.std_msgs_String);
%                r.fileWrite_pub=rospublisher('/fileWrite', 'audio_receiver/audioFile');
%                 r.playFile_pub=rospublisher('/playFile', rostype.std_msgs_String);
            end
        end
        
        function shutdown(r)
            if r.sim
                stop(r.update_timer);
                delete(r.update_timer);
			else
				try
					r.ws=0;%.delete;
                    rosshutdown%THE END!
				catch e
					error('Tried to shutdown robot, but not connected');
				end
            end
        end
		
		function forksUp(r)
			if r.sim
                fprintf('forks up !\n');
				%warning('No forks in simulation... yet');
			else
				if r.ws%.isvalid
					msg = rosmessage(r.fork_pub);
					msg.Data = 180;
					%obj.context(r.fork_pub,msg);
                    send(r.fork_pub,msg);
				else
					error('Robot Connection is not valid')
				end
			end
		end		
		
		function forksDown(r)
			if r.sim
				fprintf('forks down !\n');
                %warning('No forks in simulation... yet');
			else
				if r.ws%.isvalid
					msg = rosmessage(r.fork_pub);
					msg.Data = 0;
					send(r.fork_pub,msg);
				else
					error('Robot Connection is not valid')
				end
			end
		end	
		
		function startLaser(r)
			if r.sim
				r.sim_laser_on = true;
                if ~r.has_map
                    warning('No map, are you sure you want to run the simulated laser?');
                end
			else
				if r.ws%.isvalid
					msg = rosmessage(r.laser_pub);
					msg.Data = 1;
					send(r.laser_pub,msg);
				else
					error('Robot Connection is not valid')
				end
			end
		end

		function stopLaser(r)
			if r.sim
				r.sim_laser_on = false;
			else
				if r.ws%.isvalid
					msg = rosmessage(r.laser_pub);
					msg.Data = 0;
					send(r.laser_pub,msg);
				else
					error('Robot Connection is not valid')
				end
			end
        end
        
		function sendVelocity(r, v_l, v_r)
            if (isnan(v_l) || isnan(v_r))
                r.stop();
                error('You sent a NaN velocity, robot stopping');            
            end
			if( abs(v_l) > r.max_vel || abs(v_r) > r.max_vel)
                %fprintf('raspbot: velocity was limited\n');
				if(r.limit_wheel_speeds)
					oldv_l = v_l;
					oldv_r = v_r;
					if(abs(v_l) > abs(v_r))
						v_l = r.max_vel * (v_l/abs(v_l));
						v_r = oldv_r * (v_l /oldv_l);
					else
						v_r = r.max_vel* (v_r/abs(v_r));
						v_l = oldv_l * (v_r/oldv_r);
					end
				else
					error(['Max Vel is ' num2str(r.max_vel) ...
						', you sent ' num2str(max(abs([v_l,v_r])))]);
				end
			end
			if(r.sim)
                %fprintf('raspbot: sending velocity %f %f\n',v_l,v_r);
                % Fire sim robot up again if this is the first command to go out
                % Doing so may eliminate dead time from odometry?
                if(~r.inited)
                    r.sim_robot.fireUpForRealTime();
                    r.inited = 1;
                end
                r.fifoMutex = 1;
				r.sim_robot.sendVelocity(v_l, v_r);
                r.fifoMutex = 0;
				r.dist_since_cmd = 0;
            else
                t = toc(r.vel_timer);
                  
                if((t - r.last_vel_time) > r.min_vel_period)
                
                    msg = rosmessage(r.vel_pub);

                    msg.Linear.X = ((v_l + v_r)/2);
                    msg.Linear.Y = 0;
                    msg.Linear.Z = 0;

                    msg.Angular.X = 0;
                    msg.Angular.Y = 0;
                    %was v_l - v_r
                    msg.Angular.Z = ((v_r - v_l)/robotKinematicModel.W);

                    send(r.vel_pub,msg);
                    r.last_vel_time = t;
                end
			end
			r.vel_cmd = [v_l v_r];
        end
		
        function stop(r)
            if(r.sim)
                %fprintf('raspbot: sending stop\n');
                r.fifoMutex = 1;
				r.sim_robot.sendVelocity(0.0, 0.0);
                r.fifoMutex = 0;
				r.dist_since_cmd = 0;
                r.vel_cmd = [0.0 0.0];
            else
                msg = rosmessage(r.vel_pub);

                msg.Linear.X = 0;
                msg.Linear.Y = 0;
                msg.Linear.Z = 0;

                msg.Angular.X = 0;
                msg.Angular.Y = 0;
                msg.Angular.Z = 0;

                send(r.vel_pub,msg);
            end
        end
        
		function manual_update(r)
			toc(r.last_update)
			if( toc(r.last_update) > 1/r.sim_freq)
				r.simUpdate(0,0);
			end
        end
        
        function say(r, str)
             if (r.sim)
                 warning('No speakers in simulation');
             elseif ischar(str)
                 msg = rosmessage(r.speech_pub);
                 msg.Data = str;
                 send(r.speech_pub,msg);
             else
                 warning('Function input must be a character array, delim by prime');
             end
         end
         
%          function play(r, str)
%              if (r.sim)
%                  [y, f] = audioread(str);
%                  sound(y,f);
%              else
%                  msg = rosmessage(r.playFile_pub);
%                  msg.Data = str;
%                  send(r.playFile_pub,msg);
%              end
%          end
%         
%          function sendSoundFile(r, localPath, remotePath)
%             if (r.sim)
%                 return
%             elseif isempty(localPath(localPath == '.'))
%                 warning('Did you remember to add .wav to the local path?')
%             elseif isempty(remotePath(remotePath == '.'))
%                 warning('Did you remember to add .wav to the remote path?')
%             elseif localPath(end-2:end) ~= 'wav'
%                 warning('File must be a .wav file')
%             else
%                 [data, ~] = audioread(localPath, 'native');
%                 data = int16(data);
%                 msg = rosmessage(r.fileWrite_pub);
%                 msg.FileName = filename;
%                 msg.FileData = data(:,1);
%                 send(r.fileWrite_pub,msg);
%             end
%          end
         
         function image = captureImage(r)
             img = r.camera.LatestMessage;
             img.Format = 'bgr8; jpeg compressed bgr8';
             image = readImage(img);
             imshow(image)
         end
        
        % Add a map for use in laser simulation
		function genMap(r, obs)
			fig_num = get(r.ah,'Parent');
			r.map = lineMap(obs);
			r.has_map = true;
        end		
    
        % Change which display list is being shown
        function togglePlot(r)
            if ishandle(r.ph(1))
                hfig = get(get(r.ph(1),'parent'),'parent');
                hState = get(hfig,'visible');
                switch hState
                    case 'on'
                        set(hfig,'visible','off');
                    case 'off'
                        set(hfig,'visible','on');
                end
            end
        end
        
    end
	
	methods (Hidden = true, Access = 'private')
        
        function simUpdate(r,caller,event)
            if ~r.fifoMutex
                r.sim_robot.sendVelocity(r.vel_cmd(1), r.vel_cmd(2));
            end
			r.sim_robot.updateState();
			
			%Need to update to return in terms of metres
			el = (r.sim_robot.encoders.LatestMessage.Vector.X- r.last_l);
			%el = (r.sim_robot.encoders.LatestMessage.Left - r.last_l)/1000;
			
			%Same update needed as above.
			er = (r.sim_robot.encoders.LatestMessage.Vector.Y - r.last_r);
			%er = (r.sim_robot.encoders.LatestMessage.Right - r.last_r)/1000;
			
			%Need to update for new API
			r.last_l = r.sim_robot.encoders.LatestMessage.Vector.X;
			%r.last_l = r.sim_robot.encoders.LatestMessage.Left;
			
			r.last_r = r.sim_robot.encoders.LatestMessage.Vector.Y;
			%r.last_r = r.sim_robot.encoders.LatestMessage.Right;
			
			d = max(abs(el),abs(er));
			r.dist_since_cmd = r.dist_since_cmd + d;
			
			%stop the robot if it hasn't been commanded and has gone
			%max_dist
			if(r.dist_since_cmd > r.max_dist)
					r.vel_cmd = [0 0];
			end

			%Every third update, update the encoders
			if(r.enc_cnt == r.enc_prescaler)
				r.encoders.LatestMessage = r.sim_robot.encoders.LatestMessage;
				r.encoders.LatestMessage.Header.Stamp = r.timestamp();
				r.encoders.publish();
				r.enc_cnt = 0;
			end
			
			if(r.laser_cnt == r.laser_prescaler)
				if r.sim_laser_on
                    if r.has_map
                        l_pose = r.sim_robot.pose;
                        l_pose(1:2) = l_pose(1:2) + ...
                            robotKinematicModel.laser_l.*[cos(l_pose(3)); sin(l_pose(3))];
                        
                        [r.laser.LatestMessage.Ranges,~] = ...
                              r.map.raycast(l_pose,r.laser_range,deg2rad(-5:354)); 
%                             r.map.raycast(l_pose,r.laser_range,deg2rad(0:359));
% Added 5 deg CLOCKWISE offset to simulated LIDAR to reflect the offset  present on the physical robots 
% Max Hu 9/19/17
                    else
                        r.laser.LatestMessage.Ranges = zeros(360,1);
                    end
                end
                r.laser.LatestMessage.Header.Stamp = r.timestamp();
                r.laser.publish();
				r.laser_cnt = 0;
			end
			
			if(r.plot_cnt == r.plot_prescaler)
				r.plot();
				drawnow;
				r.plot_cnt = 0;
			end
			
			r.enc_cnt   = r.enc_cnt+1;
			r.laser_cnt = r.laser_cnt+1;
			r.plot_cnt  = r.plot_cnt+1;
			
			%start the timer for to check timing
			r.last_update = tic();
		end	
		
		function stamp = timestamp(r)
			t = toc(r.stamper);
			stamp.Sec = floor(t);
			stamp.Nsec = (t - floor(t)) * 1000000000;
		end

		function plot(r)
			
            bpts = robotKinematicModel.bodyGraph();
            bx = bpts(1,:);
            by = bpts(2,:);
            
			x = r.sim_robot.pose(1);
			y = r.sim_robot.pose(2);
			theta = r.sim_robot.pose(3);
			
			update_plot = true;
			if( x == r.last_x && y == r.last_y && theta == r.last_th)
				update_plot = false;
			end
			r.last_x = x;
			r.last_y = y;
			r.last_th = theta;
            
            px = x + (cos(theta)*bx - sin(theta)*by);
			py = y + (sin(theta)*bx + cos(theta)*by);
            
			if ishandle(r.ph(1))
				if(update_plot)
					set(r.ph(1),'XData',px,'YData',py);
                    set(r.ph(2),'XData',[get(r.ph(2),'XData') x],...
						'YData',[get(r.ph(2),'YData') y]);
				end
			else
				figure();
				r.ah = gca();
				r.ph(1) = plot(px,py,'-','LineWidth',2,'Color',[.02 .3 .05]);
                hold on
				r.ph(2) = plot(x,y,'-','LineWidth',1,'Color',[.4 .4 .9]);
				hold off;
				% axis([x+[-1 1] y+[-1 1]]);
				axis equal
				grid on
			end
			
			if r.has_map
				r.map.update;
			end
        end
        
        function delete(r)
            % Destructor
        end
    end
end
