function Zact = getZact(obj,input)
    arguments
        obj
        input.model
    end
    Zact = obj.nullJc_jointsAngVel_PJ(input.model.selector.indexes_motorsAngVel_from_jointsAngVel,:);
end
