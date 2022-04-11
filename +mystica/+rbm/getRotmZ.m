function R = getRotmZ(alpha)
    switch class(alpha)
        case 'casadi.SX'
            R = casadi.SX(3,3);
        case 'casadi.MX'
            R = casadi.MX(3,3);
        otherwise
            R = zeros(3,3);
    end
    R(3,3) =  1;
    R(1,1) =  cos(alpha);
    R(1,2) = -sin(alpha);
    R(2,1) =  sin(alpha);
    R(2,2) =  cos(alpha);
end
