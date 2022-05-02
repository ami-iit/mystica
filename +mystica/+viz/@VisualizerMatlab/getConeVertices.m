function vertices = getConeVertices(obj,mBodyPosQuat_0,i)

    index      = obj.model.joints{i}.getJointConnectionDetails();
    tform_p_pj = obj.model.linksAttributes{index.parent}.tform_b_j{index.cPointParent};
    tform_0_p  = mystica.rbm.getTformGivenPosQuat(mBodyPosQuat_0(obj.model.linksAttributes{index.parent}.selector.indexes_linkPosQuat_from_mBodyPosQuat));
    tform_0_pj = tform_0_p * tform_p_pj;
    vertices   = [ -obj.structJointShared.coneGeometry.vertices(:,1) ...
                    obj.structJointShared.coneGeometry.vertices(:,2:3)*tan(obj.model.joints{i}.limitRoM)/tan(obj.stgsVisualizer.joint.cone.angleIn)];
    vertices   = obj.getCoordinatesVerticesSTL('tform_0_originSTL',tform_0_pj,'vertices',vertices,'scale',obj.stgsVisualizer.joint.cone.dim);

end
