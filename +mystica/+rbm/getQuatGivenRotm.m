function quat = getQuatGivenRotm(rotm)
    
    tol = 1e-7;
    
    qw = ( rotm(1,1) + rotm(2,2) + rotm(3,3) + 1.0);
    qx = ( rotm(1,1) - rotm(2,2) - rotm(3,3) + 1.0);
    qy = (-rotm(1,1) + rotm(2,2) - rotm(3,3) + 1.0);
    qz = (-rotm(1,1) - rotm(2,2) + rotm(3,3) + 1.0);
    
    qw = qw*(qw>0);
    qx = qx*(qx>0);
    qy = qy*(qy>0);
    qz = qz*(qz>0);
    
    if qw >= qx && qw >= qy && qw >= qz
        qw = sqrt(qw);
        qx = (rotm(3,2) - rotm(2,3)) / (2 * qw);
        qy = (rotm(1,3) - rotm(3,1)) / (2 * qw);
        qz = (rotm(2,1) - rotm(1,2)) / (2 * qw);
        qw = qw/2;
    elseif qx >= qw && qx >= qy && qx >= qz
        qx = sqrt(qx);
        qw = (rotm(3,2) - rotm(2,3)) / (2 * qx);
        qy = (rotm(2,1) + rotm(1,2)) / (2 * qx);
        qz = (rotm(3,1) + rotm(1,3)) / (2 * qx);
        qx = qx / 2;
    elseif qy >= qw && qy >= qx && qy >= qz
        qy = sqrt(qy);
        qw = (rotm(1,3) - rotm(3,1)) / (2 * qy);
        qx = (rotm(2,1) + rotm(1,2)) / (2 * qy);
        qz = (rotm(2,3) + rotm(3,2)) / (2 * qy);
        qy = qy/2;
    elseif qz >= qw && qz >= qx && qz >= qy
        qz = sqrt(qz);
        qw = (rotm(2,1) - rotm(1,2)) / (2 * qz);
        qx = (rotm(3,1) + rotm(1,3)) / (2 * qz);
        qy = (rotm(2,3) + rotm(3,2)) / (2 * qz);
        qz = qz/2;
    end
    
    s_inv = 1;
    if abs(qw) > tol
        s_inv = sign(qw);
    elseif abs(qx) > tol
        s_inv = sign(qx);
    elseif abs(qy) > tol
        s_inv = sign(qy);
    elseif abs(qz) > tol
        s_inv = sign(qz);
    end
    
    qw = qw/s_inv;
    qx = qx/s_inv;
    qy = qy/s_inv;
    qz = qz/s_inv;
    
    quaternionNorm = qw * qw + qx * qx + qy * qy + qz * qz;
    
    quat = [qw;qx;qy;qz] / quaternionNorm;
    
end
