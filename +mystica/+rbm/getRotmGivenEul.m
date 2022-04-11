function rotm = getRotmGivenEul(fun,angle)
    arguments (Repeating)
        fun    char
        angle
    end

    % rotm = R(1) * ... * R(n) * I
    rotm = eye(3);

    for i = length(fun) : -1 : 1
        R = str2func(fun{i});
        rotm = R(angle{i}) * rotm;
    end

end

function rotm = rx(angle)
    rotm = mystica.rbm.getRotmX(angle);
end
function rotm = ry(angle)
    rotm = mystica.rbm.getRotmY(angle);
end
function rotm = rz(angle)
    rotm = mystica.rbm.getRotmZ(angle);
end
