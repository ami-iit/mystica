function quat = multiplyQuat(quat1,quat2)

    qw1 = quat1(1); qxyz1 = quat1(2:4);
    qw2 = quat2(1); qxyz2 = quat2(2:4);

    pw   = qw1*qw2 - qxyz1'*qxyz2;
    pxyz = qw1*qxyz2 + qw2*qxyz1 + cross(qxyz1,qxyz2);

    quat = [pw;pxyz];
end
