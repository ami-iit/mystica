function q = fixSignQuat(quat)
    qw = quat(1);
    q = -quat*(qw<0)+quat*(qw>=0);
end
