function theta = logQuat(quat,input)
    arguments
        quat
        input.fixSign logical = false
    end
    if input.fixSign
        quat = mystica.rbm.fixSignQuat(quat);
    end

    qw = quat(1); qxyz = quat(2:4);
    normV = norm(qxyz,2)+eps;
    theta = 2 * qxyz * atan2(normV,qw)/normV;

end
