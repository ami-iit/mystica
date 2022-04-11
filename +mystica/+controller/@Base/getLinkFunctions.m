function getLinkFunctions(obj,model,funDesiredShape,invertNormal)
    if invertNormal == 1
        signGrad = -1;
    else
        signGrad = 1;
    end

    mBodyPosQuat_0 = casadi.SX.sym('mBodyPosQuat',model.constants.mBodyPosQuat,1);

    posQuat  = casadi.SX.sym('posQuat',7,1);
    rotm_0_b = casadi.SX.sym('rotm_0_b',3,3);
    pos_0_b  = casadi.SX.sym('pos_0_b',3,1);
    t        = casadi.SX.sym('t',1);

    xLink      = pos_0_b(1);
    yLink      = pos_0_b(2);
    zLink      = pos_0_b(3);
    normalLink = rotm_0_b(1:3,3);

    %% Normals

    funExp = funDesiredShape(xLink,yLink,t);
    funImp = funExp - pos_0_b(3);

    grad = signGrad * gradient(funImp,pos_0_b);

    normalDes    = grad/norm(grad);
    dotNormalDes = jacobian(normalDes,t);
    omegaNormDes = cross(dotNormalDes,normalDes);

    obj.csdFn.normalDes = casadi.Function('normalDes',{pos_0_b,t},{normalDes});

    %% LinkAngVelStar

    Kaligned  = obj.stgsController.costFunction.gainLinkAngVelStarAligned;
    Kopposite = obj.stgsController.costFunction.gainLinkAngVelStarOpposite;

    Ka = 2*Kaligned/(Kopposite-Kaligned)*Kopposite;
    Kb = 2*Kaligned/(Kopposite-Kaligned);

    Kgain = Ka/(1+dot(normalDes,normalLink)+Kb);

    linkAngVelStar = Kgain * mystica.utils.skew(normalLink) * normalDes + ...
        (omegaNormDes - (dot(omegaNormDes,normalLink)*normalLink))*obj.stgsController.costFunction.useFeedForwardTermLinkAngVelStar;

    linkAngVelStar_csdFn_posRotm = casadi.Function('linkAngVelStar',{pos_0_b,rotm_0_b,t},{linkAngVelStar});
    linkAngVelStar_csdFn_posQuat = casadi.Function('linkAngVelStar',{posQuat,t},{linkAngVelStar_csdFn_posRotm(mystica.rbm.getPosGivenPosQuat(posQuat),mystica.rbm.getRotmGivenPosQuat(posQuat),t)});
    mBodyAngVelStar_csdFn_matrixPosQuat = linkAngVelStar_csdFn_posQuat.map(model.nLink);
    obj.csdFn.mBodyAngVelStar = casadi.Function('mBodyAngVelStar',{mBodyPosQuat_0,t},{reshape(mBodyAngVelStar_csdFn_matrixPosQuat(reshape(mBodyPosQuat_0,7,model.nLink),t),3*model.nLink,1)});

    %% Error

    linkErrorOrientationNormal = acos(dot(normalDes,normalLink));
    linkErrorPositionNormal    = zLink-funExp;

    linkErrorOrientationNormal_csdFn_posRotm = casadi.Function('linkErrorOrientationNormal',{pos_0_b,rotm_0_b,t},{linkErrorOrientationNormal});
    linkErrorOrientationNormal_csdFn_posQuat = casadi.Function('linkErrorOrientationNormal',{posQuat,t},...
        {linkErrorOrientationNormal_csdFn_posRotm(mystica.rbm.getPosGivenPosQuat(posQuat),mystica.rbm.getRotmGivenPosQuat(posQuat),t)});
    mBodyErrorOrientationNormal_csdFn_matrixPosQuat = linkErrorOrientationNormal_csdFn_posQuat.map(model.nLink);
    obj.csdFn.mBodyErrorOrientationNormal = casadi.Function('mBodyErrorOrientationNormal',{mBodyPosQuat_0,t},{reshape(mBodyErrorOrientationNormal_csdFn_matrixPosQuat(reshape(mBodyPosQuat_0,7,model.nLink),t),model.nLink,1)});

    linkErrorPositionNormal_csdFn_posRotm = casadi.Function('linkErrorPositionNormal',{pos_0_b,rotm_0_b,t},{linkErrorPositionNormal});
    linkErrorPositionNormal_csdFn_posQuat = casadi.Function('linkErrorPositionNormal',{posQuat,t},...
        {linkErrorPositionNormal_csdFn_posRotm(mystica.rbm.getPosGivenPosQuat(posQuat),mystica.rbm.getRotmGivenPosQuat(posQuat),t)});
    mBodyErrorPositionNormal_csdFn_matrixPosQuat = linkErrorPositionNormal_csdFn_posQuat.map(model.nLink);
    obj.csdFn.mBodyErrorPositionNormal = casadi.Function('mBodyErrorPositionNormal',{mBodyPosQuat_0,t},{reshape(mBodyErrorPositionNormal_csdFn_matrixPosQuat(reshape(mBodyPosQuat_0,7,model.nLink),t),model.nLink,1)});

end
