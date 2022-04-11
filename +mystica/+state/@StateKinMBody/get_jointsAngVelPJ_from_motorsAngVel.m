function jointsAngVel_PJ = get_jointsAngVelPJ_from_motorsAngVel(obj,input)
    arguments
        obj
        input.motorsAngVel    (:,1)
        input.model
        input.regTermDampPInv (1,1)
    end

    Zact            = obj.getZact('model',input.model);
    invZact         = mystica.utils.pinvDamped(Zact,input.regTermDampPInv);
    jointsAngVel_PJ = obj.nullJc_jointsAngVel_PJ * invZact * input.motorsAngVel;

end
