function posQuat = getPosQuatGivenTform(tform)

    posQuat = [tform(1:3,4);mystica.rbm.getQuatGivenRotm(tform(1:3,1:3))];

end
