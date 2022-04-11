function createSelector(obj)
    
    %% indexes
    
    obj.selector.indexes_mBodyAngVel_from_mBodyTwist = sort(reshape(...
        (obj.constants.linkAngVel + (1:obj.constants.linkAngVel)) + ...
        obj.constants.linkTwist*transpose(0:obj.nLink-1),1,[]));

    obj.selector.indexes_mBodyLinVel_from_mBodyTwist = sort(reshape(...
        ( 1:obj.constants.linkPos) + ...
        obj.constants.linkTwist*transpose(0:obj.nLink-1),1,[]));

    obj.selector.indexes_mBodyPos_from_mBodyPosQuat = sort(reshape(...
        (1:obj.constants.linkPos) + ...
        obj.constants.linkPosQuat*transpose([0:obj.nLink-1]),1,[]));

    obj.selector.indexes_mBodyPosQuat_from_mBodyPosVel =  1:obj.constants.mBodyPosQuat;
    obj.selector.indexes_mBodyTwist_from_mBodyPosVel   = (1:obj.constants.mBodyTwist) + obj.constants.mBodyPosQuat;
    
    booleanJointsAngVelActuated    = zeros(obj.constants.jointsAngVel,1);
    booleanJointsAngVelPassive     = zeros(obj.constants.jointsAngVel,1);
    booleanJointsAngVelConstrained = zeros(obj.constants.jointsAngVel,1); 
    for j = 1 : obj.nJoint
        booleanJointsAngVelActuated(   obj.joints{j}.selector.indexes_jAngVel_from_jointsAngVel) =  obj.joints{j}.axesActuated;
        booleanJointsAngVelPassive(    obj.joints{j}.selector.indexes_jAngVel_from_jointsAngVel) = ~obj.joints{j}.axesActuated &  obj.joints{j}.axesRotation;
        booleanJointsAngVelConstrained(obj.joints{j}.selector.indexes_jAngVel_from_jointsAngVel) = ~obj.joints{j}.axesActuated & ~obj.joints{j}.axesRotation;
    end
    obj.selector.indexes_motorsAngVel_from_jointsAngVel      = find(booleanJointsAngVelActuated);
    obj.selector.indexes_passiveAngVel_from_jointsAngVel     = find(booleanJointsAngVelPassive);
    obj.selector.indexes_constrainedAngVel_from_jointsAngVel = find(booleanJointsAngVelConstrained);
    
    obj.selector.indexes_constrainedAngVel_from_JcV = [];
    obj.selector.indexes_constrainedLinVel_from_JcV = [];
    
    %% Matrix
    
    obj.selector.matrix_mBodyAngVel_from_mBodyTwist = sparse(obj.constants.mBodyAngVel,obj.constants.mBodyTwist);
    obj.selector.matrix_mBodyAngVel_from_mBodyTwist(:,obj.selector.indexes_mBodyAngVel_from_mBodyTwist) = eye(length(obj.selector.indexes_mBodyAngVel_from_mBodyTwist));

    obj.selector.matrix_motorsAngVel_from_jointsAngVel      = sparse(sum(booleanJointsAngVelActuated)   ,obj.constants.jointsAngVel);
    obj.selector.matrix_passiveAngVel_from_jointsAngVel     = sparse(sum(booleanJointsAngVelPassive)    ,obj.constants.jointsAngVel);
    obj.selector.matrix_constrainedAngVel_from_jointsAngVel = sparse(sum(booleanJointsAngVelConstrained),obj.constants.jointsAngVel);
    
    obj.selector.matrix_motorsAngVel_from_jointsAngVel(     :,obj.selector.indexes_motorsAngVel_from_jointsAngVel)      = eye(length(obj.selector.indexes_motorsAngVel_from_jointsAngVel));
    obj.selector.matrix_passiveAngVel_from_jointsAngVel(    :,obj.selector.indexes_passiveAngVel_from_jointsAngVel)     = eye(length(obj.selector.indexes_passiveAngVel_from_jointsAngVel));
    obj.selector.matrix_constrainedAngVel_from_jointsAngVel(:,obj.selector.indexes_constrainedAngVel_from_jointsAngVel) = eye(length(obj.selector.indexes_constrainedAngVel_from_jointsAngVel));
    
end
