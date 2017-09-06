% Sets/Saves a new Robot State from which Odometry is Collected (overwrites 
% previous).
% Returns vector containing new state variables.
function state = startTrip(obj)
    obj.trip_startTime = tic;
    obj.init_enc_l = obj.core.encoders.LatestMessage.Vector.X;
    obj.init_enc_r = obj.core.encoders.LatestMessage.Vector.Y;
    
    state = [
            obj.trip_startTime ...
            obj.init_enc_l ...
            obj.init_enc_r
            ];
end