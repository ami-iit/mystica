function tform = getTformGivenPosQuat(posQuat)

    tform = [mystica.rbm.getRotmGivenPosQuat(posQuat) posQuat(1:3) ; zeros(1,3) 1];

end
