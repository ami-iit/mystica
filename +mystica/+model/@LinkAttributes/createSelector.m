function selector = createSelector(obj,constants)
%CREATESELECTOR Summary of this function goes here
%   Detailed explanation goes here

    selector.indexes_linkPosQuat_from_mBodyPosQuat = (1:constants.linkPosQuat)+(obj.index-1)*constants.linkPosQuat;
    selector.indexes_linkAngVel_from_mBodyTwist    = (1:constants.linkAngVel) +(obj.index-1)*constants.linkTwist  +constants.linkPos;
    selector.indexes_linkAngVel_from_mBodyAngVel   = (1:constants.linkAngVel) +(obj.index-1)*constants.linkAngVel;
    selector.indexes_linkLinVel_from_mBodyTwist    = (1:constants.linkPos)    +(obj.index-1)*constants.linkTwist;
    selector.indexes_linkTwist_from_mBodyTwist     = (1:constants.linkTwist)  +(obj.index-1)*constants.linkTwist;


    selector.matrix_linkAngVel_from_mBodyTwist = sparse(constants.linkAngVel,constants.mBodyTwist);
    selector.matrix_linkAngVel_from_mBodyTwist(1:constants.linkAngVel,selector.indexes_linkAngVel_from_mBodyTwist) = eye(constants.linkAngVel);

%   selector.matrix_linkLinVel_from_mBodyTwist = sparse(constants.linkPos,constants.mBodyTwist);
%   selector.matrix_linkLinVel_from_mBodyTwist(1:constants.linkPos,selector.indexes_linkLinVel_from_mBodyTwist) = eye(constants.linkPos);

    selector.matrix_linkTwist_from_mBodyTwist = sparse(constants.linkTwist,constants.mBodyTwist);
    selector.matrix_linkTwist_from_mBodyTwist(1:constants.linkTwist,selector.indexes_linkTwist_from_mBodyTwist) = eye(constants.linkTwist);

end
