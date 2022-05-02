function R = getRotmY(alpha)
    switch class(alpha)
        case 'casadi.SX'
            R = casadi.SX(3,3);
        case 'casadi.MX'
            R = casadi.MX(3,3);
        otherwise
            R = zeros(3,3);
    end
    R(2,2) =  1;
    R(1,1) =  cos(alpha);
    R(1,3) =  sin(alpha);
    R(3,1) = -sin(alpha);
    R(3,3) =  cos(alpha);
end
