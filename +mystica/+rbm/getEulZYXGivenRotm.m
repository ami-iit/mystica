function eul = getEulZYXGivenRotm(rotm)
    % For documentation http:%www.geometrictools.com/Documentation/EulerAngles.pdf

    if rotm(3,1) < +1
        if rotm(3,1) > -1
            thetaY = asin(-rotm(3,1) ) ;
            thetaZ = atan2( rotm(2,1) , rotm(1,1) ) ;
            thetaX = atan2( rotm(3,2) , rotm(3,3) ) ;
        else % rotm(3,1) = -1
            % Not a unique solution: thetaX - thetaZ = atan2(-rotm(2,3) , rotm(2,2) )
            thetaY = +pi / 2;
            thetaZ = -atan2(-rotm(2,3) , rotm(2,2) ) ;
            thetaX = 0 ;
        end
    else % rotm(3,1) = +1
        % Not a unique solution: thetaX + thetaZ = atan2(-rotm(2,3) , rotm(2,2) )
        thetaY = -pi / 2;
        thetaZ = atan2(-rotm(2,3) , rotm(2,2) ) ;
        thetaX = 0 ;
    end

    eul = [thetaZ;thetaY;thetaX];

end
