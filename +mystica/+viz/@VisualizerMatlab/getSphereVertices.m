function vertices = getSphereVertices(obj,mBodyPosQuat_0,j)

    index      = obj.model.joints{j}.getJointConnectionDetails();
    tform_p_pj = obj.model.linksAttributes{index.parent}.tform_b_j{index.cPointParent};
    tform_0_p  = mystica.rbm.getTformGivenPosQuat(mBodyPosQuat_0(obj.model.linksAttributes{index.parent}.selector.indexes_linkPosQuat_from_mBodyPosQuat));
    tform_0_pj = tform_0_p * tform_p_pj;
    selJAngVel = obj.model.joints{j}.selector.indexes_jAngVel_from_jointsAngVel;
    ratio      = max(abs(obj.data.jointsAngVel_PJ(selJAngVel,obj.indexIterationVis)))/max(obj.model.joints{j}.limitJointVel);
    scale      = obj.stgsVisualizer.joint.sphere.dimMin + (obj.stgsVisualizer.joint.sphere.dimMax - obj.stgsVisualizer.joint.sphere.dimMin) * ratio;
    vertices   = obj.getCoordinatesVerticesSTL('tform_0_originSTL',tform_0_pj,'vertices',obj.structJointShared.sphereGeometry.vertices,'scale',scale);

end
