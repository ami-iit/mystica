function tform = getTformGivenPosRotm(pos,rotm)

    tform = [rotm pos; zeros(1,3) 1];

end
