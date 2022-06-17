function mBodyVelQuat_0 = get_mBodyVelQuat0_from_mBodyTwist0(obj,input)
    arguments
        obj
        input.mBodyTwist_0   (:,1)
        input.model          mystica.model.Model
        input.kBaum          (1,1)
        input.mBodyPosQuat_0_warningOnlyIfNecessary (:,1)
    end

    %     if `input.mBodyPosQuat_0` is set, the state should be updated.
    %     However this is time consuming. For the aim of this function it's not
    %     necessary to compute all the quantity state dependent.
    %
    if isfield(input,'mBodyPosQuat_0_warningOnlyIfNecessary')
        obj.mBodyPosQuat_0 = input.mBodyPosQuat_0_warningOnlyIfNecessary;
        %obj.setMBodyPosQuat('model',input.model,'mBodyPosQuat_0',input.mBodyPosQuat_0)
    end

    if isnumeric(obj.mBodyPosQuat_0) && isnumeric(input.mBodyTwist_0) && isnumeric(input.kBaum)
        mBodyVelQuat_0 = full(mystica_stateKin('get_mBodyVelQuat_from_mBodyTwist',obj.mBodyPosQuat_0,input.mBodyTwist_0,input.kBaum));
    else
        mBodyVelQuat_0 = full(obj.csdFn.get_mBodyVelQuat0_from_mBodyTwist0(obj.mBodyPosQuat_0,input.mBodyTwist_0,input.kBaum));
    end

end
