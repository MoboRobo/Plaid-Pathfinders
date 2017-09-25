%Computes the Elapsed Reference Distance of an Velocity Profile, @uref,
%from time t_i to t_f
function s = s_ref_arbitrary(uref,t_i,t_f, res)
    if (nargin < 4) %no resolution provided
        res = (t_f-t_i)/100;
    end
    s = 0;
    ts = (t_i:res:t_f);
    for i = (2:length(ts))
        s = s + res*(uref(ts(i)) + uref(ts(i-1)))/2;
    end
end