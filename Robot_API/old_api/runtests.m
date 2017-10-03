
rosshutdown
robot = raspbot('RaspBot-17');
close all

robot.startLaser();
pause(1);

dt = 0.05;

tic;

penc = [robot.encoders.LatestMessage.Vector.X, robot.encoders.LatestMessage.Vector.Y];

done = 0;

state = 0;

rposx = 0;
rposy = 0;
rang = 0;

while ~done
    
    t = toc
    
    if t < 15
        robot.sendVelocity(0.1, 0.18);
    else
        robot.stop();
        done = 1;
    end
    
    ranges = robot.laser.LatestMessage.Ranges';
    angles = linspace(robot.laser.LatestMessage.AngleMin, robot.laser.LatestMessage.AngleMax, length(ranges));
    
    lscan = [ranges.*cos(angles); ranges.*sin(angles)];
    
    lenc = [robot.encoders.LatestMessage.Vector.X, robot.encoders.LatestMessage.Vector.Y]*1.215;
    denc = lenc-penc;
    
    dang = -diff(denc)/0.090;
    dist = mean(denc);
    
    rang = wrapToPi(rang + dang);
    rposx = rposx + dist*cos(rang);
    rposy = rposy + dist*sin(rang);
    
    [rposx, rposy, rang];
    
    rotscan = [(lscan(1, :).*cos(rang) + lscan(2, :).*sin(rang)) + rposx; (-lscan(1, :).*sin(rang) + lscan(2, :).*cos(rang)) + rposy];
    
    subplot 211
    plot(lscan(1, :), lscan(2, :), '.');
    axis([-2 2 -2 2]);
    subplot 212
    hold on
    plot(rotscan(1, :), rotscan(2, :), '.');
    hold off
    axis([-2 2 -2 2]);
    
    penc = lenc;
    
    pause(dt);
end

robot.stopLaser();
robot.shutdown