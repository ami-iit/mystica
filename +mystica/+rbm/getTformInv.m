function invTform = getTformInv(tform)

    rotm  = tform(1:3,1:3);
    pos   = tform(1:3,4);
    invTform = [transpose(rotm) , -transpose(rotm)*pos ; zeros(1,3) 1];

end
