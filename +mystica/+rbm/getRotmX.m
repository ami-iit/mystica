function R = getRotmX(alpha)
    switch class(alpha)
        case 'casadi.SX'
            R = casadi.SX(3,3);
        case 'casadi.MX'
            R = casadi.MX(3,3);
        otherwise
            R = zeros(3,3);
    end
    R(1,1) =  1;
    R(2,2) =  cos(alpha);
    R(2,3) = -sin(alpha);
    R(3,2) =  sin(alpha);
    R(3,3) =  cos(alpha);
end
