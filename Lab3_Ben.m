
robot = raspbot("Raspbot-4")

robot.encoders.NewMessageFcn=@encoderEventListener;

while(true)
   pause(0.01) 
end

function encoderEventListener(handle,event)
%EncoderEventListener Invoked when new encoder data arrives.
% A MATLAB event listener for the Robot. Invoked when
% encoder data arrives.
... % Do some stuff
encoderDataTimestamp = double(event.Header.Stamp.Sec) + ...
 double(event.Header.Stamp.Nsec)/1000000000.0;
encoderFrame = encoderFrame + 1;
… % Do some more stuff
end