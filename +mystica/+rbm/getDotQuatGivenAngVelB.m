function dotQuat = getDotQuatGivenAngVelB(quat,omega_b_0b,kBaum)
    
    % qw = quat(1);
    % qx = quat(2);
    % qy = quat(3);
    % qz = quat(4);
    
    quat = quat(:);
    
    SkewBar = ...
        [0            -transpose(omega_b_0b);
        omega_b_0b    -mystica.utils.skew(omega_b_0b)];
    
    dotQuat = 0.5 * SkewBar * quat + kBaum * (1 - transpose(quat)*quat)*quat;
    
end
