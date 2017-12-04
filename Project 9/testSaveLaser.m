function testSaveLaser(robot_id, capture)
global ft;
units();

% - capture, whether to capture images first or just display them.
    if capture
        mrpl = mrplSystem(robot_id, pose(0,0,0)); 
        mrpl.rob.laserOn()

        clk = Clock();
        first_loop = 1;

        pause(1);
        images = RangeImage.empty();

        pause(5); % Wait for laser to spin up

        figure();
        counter = 0;
        while (counter < 10)
        disp("snap");
        if(first_loop)
            clk = Clock();
            first_loop = 0;
            images = mrpl.rob.hist_laser.last();
        else
            images(end+1) = mrpl.rob.hist_laser.last();
        end

        counter = counter + 1;

        mrpl.rob.hist_laser.last.plot();
        pause(5);
        beep;

        end
        save(strcat('rangeImages2'),'images');

    end % capture?

  
  persistent imgs
  function load_data()
    load(strcat('rangeImages2'),'images');
    imgs = images;
  end

    load_data();

    for i = imgs
        lidar_plotting(i);
    end
    
    function lidar_plotting(r_img)
            curr_fig = figure();
            grid on
            grid minor
            
            lidar_plot = scatter(0,0,36,0);
            set(gca, 'Xdir', 'reverse'); % Ensure Robot Y-Axis Points Left

            xlabel('Y-Position [m]');
            ylabel('X-Position [m]');
            axis(1.2*[
                -RangeImage.MAX_RANGE RangeImage.MAX_RANGE ...
                -RangeImage.MAX_RANGE RangeImage.MAX_RANGE ...
            ]);
        
            figure(curr_fig);
        
            r_img.plot(1, lidar_plot);
            
            r_img.findLineCandidates();
            
            n_lc = length(r_img.line_candidates.lengths) - 1;
            title({'LIDAR Data', strcat('LC Found: ', num2str(n_lc))});
            
            r_img.plotLineCandidates(0, gca, 1);
            
            set(curr_fig, 'Position', get(0, 'Screensize')); % Fullscreen
    end
  
end