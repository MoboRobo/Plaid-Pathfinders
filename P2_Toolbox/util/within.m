%% WITHIN
% Returns whether the given value x is (inclusively) within d units of the 
% target value y.
function b = within(x, d, y)
    below = (x < (y-d));
    above = (x > (y+d));
    b = ~(below | above);
end % #within