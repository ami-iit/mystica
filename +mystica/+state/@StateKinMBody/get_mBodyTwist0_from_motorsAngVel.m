function mBodyTwist_0 = get_mBodyTwist0_from_motorsAngVel(obj,input)
    arguments
        obj
        input.motorsAngVel    (:,1)
        input.model
        input.regTermDampPInv (1,1)
    end
    
    Zact         = obj.getZact('model',input.model);
    invZact      = mystica.utils.pinvDamped(Zact,input.regTermDampPInv);
    mBodyTwist_0 = obj.nullJc_mBodyTwist_0 * invZact * input.motorsAngVel;
    
end
