function mBodyVelQuat_0 = get_mBodyVelQuat0_from_motorsAngVel(obj,input)
    arguments
        obj
        input.motorsAngVel    (:,1)
        input.model           mystica.model.Model
        input.kBaum           (1,1)
        input.regTermDampPInv (1,1)
        input.mBodyPosQuat_0_warningOnlyIfNecessary  (:,1)
    end

    if isfield(input,'mBodyPosQuat_0_warningOnlyIfNecessary')
        obj.setMBodyPosQuat('model',input.model,'mBodyPosQuat_0',input.mBodyPosQuat_0_warningOnlyIfNecessary)
    end

    Zact         = obj.getZact('model',input.model);
    invZact      = mystica.utils.pinvDamped(Zact,input.regTermDampPInv);
    mBodyTwist_0 = obj.nullJc_mBodyTwist_0 * invZact * input.motorsAngVel;

    if isnumeric(obj.mBodyPosQuat_0) && isnumeric(mBodyTwist_0) && isnumeric(input.kBaum)
        mBodyVelQuat_0 = full(mystica_stateKin('get_mBodyVelQuat0_from_mBodyTwist0',obj.mBodyPosQuat_0,mBodyTwist_0,input.kBaum));
    else
        mBodyVelQuat_0 = full(obj.csdFn.get_mBodyVelQuat0_from_mBodyTwist0(obj.mBodyPosQuat_0,mBodyTwist_0,input.kBaum));
    end

end
