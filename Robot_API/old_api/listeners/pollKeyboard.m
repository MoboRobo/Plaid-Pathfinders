function res = pollKeyboard()
%pollKeyboard Waits until the callback says there is new data.
%   This routine is useful when you want to be able to capture and respond
%   to a keypress as soon as it arrives.
%   To use this, execute the following line:
%   kh = event.listener(gcf,'KeyPressFcn',@keyboardEventListener);
%   before calling this function.
global keypressDataReady;
global keypressKey;

keyboardDataReadyLast = keypressDataReady;
keypressDataReady = 0;
if(keyboardDataReadyLast)
    res = keypressKey;
    disp('gotOne');
else
    res = false;
end
end