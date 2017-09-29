% Incorporate path reduction algorithm like RDP?
classdef TrajectoryCurve
    properties(GetAccess = public, SetAccess = private)
        velProfile = [struct('V',0, 'om',0)];
    end % ContinuousCurve <- properties
    
    resolveData
end % 