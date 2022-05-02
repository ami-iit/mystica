function quat = getQuatGivenTform(tform)

    quat = mystica.rbm.getQuatGivenRotm(tform(1:3,1:3));

end
