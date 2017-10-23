function laserEventListenerExample(handle,event)
%laserEventListener Invoked when new laser data arrives.
%   A MATLAB event listener for the Neato Robot. Invoked when laser data
%   arrives.

global laserFrame;
global laserDataReady;

%disp('Got laser');

laserFrame = laserFrame + 1;
laserDataReady = 1;
end
