robot = raspbot();

enc_c = [];
enc_t = [];

tic;
t = toc;
while t < 10.0
    robot.sendVelocity(0.5*sin(2*t), -0.5*sin(2*t));
    
    encMsg = robot.encoders.LatestMessage;
    
    ts = double(encMsg.Header.Stamp.Sec) + double(encMsg.Header.Stamp.Nsec)/1e9;
    
    if isempty(enc_t) || enc_t(end) ~= ts
       enc_t = [enc_t ts];
       enc_c = [enc_c, [encMsg.Vector.X; encMsg.Vector.Y]]; 
    end
    t = toc;
    
    pause(0.001);
end

enc_t = enc_t - enc_t(1);
enc_c(1, :) = enc_c(1, :) - enc_c(1, 1);
enc_c(2, :) = enc_c(2, :) - enc_c(2, 1);

dist = mean(enc_c(:, end))

plot(enc_t, enc_c);

robot.stop()

robot.shutdown