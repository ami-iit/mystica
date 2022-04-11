function rotm = getRotmGivenTwoVectors(a,b)

    % rotm * a = b

    a = a/norm(a);
    b = b/norm(b);
    
    v = cross(a,b);
    s = norm(v);
    c = dot(a,b);
    
    rotm = eye(3) + mystica.utils.skew(v) + mystica.utils.skew(v)^2 * (1-c)/s^2;
    
end
