function theta = logQuat(quat)

    qw = quat(1); qxyz = quat(2:4);

    normV = norm(qxyz,2)+eps;

    theta = 2 * qxyz * atan2(normV,qw)/normV;

end
