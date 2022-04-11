function rotm = getRotmGivenPosQuat(posQuat)
    
    quat = posQuat(4:7);
    rotm = mystica.rbm.getRotmGivenQuat(quat);
    
end
