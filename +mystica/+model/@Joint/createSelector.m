function selector = createSelector(obj,constants)
%CREATESELECTOR Summary of this function goes here
%   Detailed explanation goes here
    selector.indexes_jAngVel_from_jointsAngVel = [1:constants.linkAngVel]+(obj.index-1)*constants.linkAngVel;

    selector.matrix_jAngVel_from_jointsAngVel = sparse(constants.linkAngVel,constants.jointsAngVel);
    selector.matrix_jAngVel_from_jointsAngVel(1:constants.linkAngVel,selector.indexes_jAngVel_from_jointsAngVel) = eye(constants.linkAngVel);

end
