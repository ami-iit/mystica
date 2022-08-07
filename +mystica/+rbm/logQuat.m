function theta = logQuat(quat,input)
    arguments
        quat
        input.selectQuat char {mustBeMember(input.selectQuat,{'positive','minDistance','normal'})} = 'normal'
    end
    qw = quat(1); qxyz = quat(2:4);
    switch input.selectQuat
        case 'normal'
            normV = norm(qxyz,2)+eps;
            theta = 2 * qxyz * atan2(normV,qw)/normV;
        case 'positive'
            quat  = -quat*(qw<0)+quat*(qw>=0);
            theta = mystica.rbm.logQuat(quat,'selectQuat','normal');
        case 'minDistance'
            theta1 = mystica.rbm.logQuat( quat,'selectQuat','normal'); n1 = norm(theta1,2);
            theta2 = mystica.rbm.logQuat(-quat,'selectQuat','normal'); n2 = norm(theta2,2);
            theta = theta1*(n1<=n2)+theta2*(n2<n1);
    end
end
