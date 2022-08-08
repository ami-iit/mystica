function rotm = getRotmGivenQuat(quat)

    qw   = quat(1);
    qxyz = quat(2:4);

    rotm = eye(3)+2*qw*mystica.utils.skew(qxyz)+2*mystica.utils.skew(qxyz)^2;

end
