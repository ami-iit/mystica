function updateLinkState(obj,model)
    arguments
        obj   mystica.state.StateKinMBody
        model mystica.model.Model
    end
    for i = 1 : model.nLink
        obj.linksState{i}.setLinkStateGivenLinkPosQuat(obj.mBodyPosQuat_0(model.linksAttributes{i}.selector.indexes_linkPosQuat_from_mBodyPosQuat));
    end
end
