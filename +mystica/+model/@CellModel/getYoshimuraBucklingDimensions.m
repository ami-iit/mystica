function [angleH,angleV,h] = getYoshimuraBucklingDimensions(d,m)
    
    c = d * sin(pi/m);
    e = d/2*(1-cos(pi/m));
    
    h = sqrt(c^2-c^2/4-e^2);
    
    pointA = [d/2-e;c/2;h];
    pointB = [d/2;0;0];
    pointC = [d/2-e;-c/2;h];
    pointD = mystica.rbm.getRotmGivenEul('rz',-2*pi/m)*pointB;
    
    n1 = cross(pointA-pointB,pointA-pointC);
    n1 = n1/norm(n1);
    n2 = cross(pointC-pointB,pointC-pointD);
    n2 = n2/norm(n2);
    
    n3 = -[0;0;1];
    
    angleH = acos(dot(n1,n2));
    angleV = -2*(pi/2-acos(dot(n1,n3)));
    
end
