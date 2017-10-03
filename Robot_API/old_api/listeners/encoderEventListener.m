function encoderEventListener(handle,event)
%   encoderEventListener Invoked when new encoder data arrives.
%   A MATLAB event listener for the Robot. Invoked when encoder data
%   arrives.

global encoderFrame;
global encoderDataReady;
global encoderDataTimeStamp;
global encoderDataStarted;
global encoderDataTimeStart;

if(~isscalar(encoderDataStarted))
    encoderDataStarted = true;
    encoderDataTimeStart = double(event.Header.Stamp.Sec);
    disp('Encoder Event Listener is Up\n');
end

encoderDataTimeStamp = double(event.Header.Stamp.Sec) - ...
    double(encoderDataTimeStart) + ...
	double(event.Header.Stamp.Nsec)/1000000000.0;
%fprintf('in event listener\n');

encoderFrame = encoderFrame + 1;
encoderDataReady = 1;
end
