function setMBodyPosQuat(obj,input)
    arguments
        obj                  mystica.state.StateKinMBody
        input.mBodyPosQuat_0 (:,1)
        input.model          mystica.model.Model
    end
    
    obj.mBodyPosQuat_0 = input.mBodyPosQuat_0;
    obj.Jc = sparse(mystica_stateKin('Jc',obj.mBodyPosQuat_0)); % obj.csdFn.Jc(obj.mBodyPosQuat_0)
    obj.referenceConversion.from_mBodyTwist0_2_jointsAngVelPJ = sparse(mystica_stateKin('rC_from_mBodyTwist0_2_jointsAngVelPJ',obj.mBodyPosQuat_0)); % obj.csdFn.rC_from_mBodyTwist0_2_jointsAngVelPJ(obj.mBodyPosQuat_0)
    obj.nullJc_mBodyTwist_0 = obj.nullEvaluator.compute(obj.Jc);
    obj.nullJc_jointsAngVel_PJ = obj.referenceConversion.from_mBodyTwist0_2_jointsAngVelPJ * obj.nullJc_mBodyTwist_0;
    obj.linIndRowJc = obj.nullEvaluator.getLinIndRow;
    
end
