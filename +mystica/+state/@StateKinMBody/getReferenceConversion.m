function getReferenceConversion(obj,model)
    %GETREFERENCECONVERSION Summary of this function goes here
    %   Detailed explanation goes here
    arguments
        obj   mystica.state.StateKinMBody
        model mystica.model.Model
    end
    
    rC_from_mBodyTwist0_2_jointsAngVelPJ{model.nJoint} = {};
    
    rC_from_jointsAngVelPJ_2_jointsAngVel0{model.nJoint} = {};
    
    for j = 1 : model.nJoint
        index = model.joints{j}.getJointConnectionDetails();
        
        sel_angVel_c = model.linksAttributes{index.child }.selector.matrix_linkAngVel_from_mBodyTwist;
        sel_angVel_p = model.linksAttributes{index.parent}.selector.matrix_linkAngVel_from_mBodyTwist;
        
        rotm_p_0  = transpose(mystica.rbm.getRotmGivenTform(obj.linksState{index.parent}.csdFn.tform_0_b(obj.csdSy.mBodyPosQuat_0)));
        rotm_pj_p = transpose(mystica.rbm.getRotmGivenTform(model.linksAttributes{index.parent}.tform_b_j{index.cPointParent}));
        
        rC_from_mBodyTwist0_2_jointsAngVelPJ{j}   = rotm_pj_p * rotm_p_0 * (sel_angVel_c - sel_angVel_p);
        rC_from_jointsAngVelPJ_2_jointsAngVel0{j} = transpose(rotm_pj_p*rotm_p_0);
        
    end
    
    obj.csdSy.rC_from_mBodyTwist0_2_jointsAngVelPJ = vertcat(rC_from_mBodyTwist0_2_jointsAngVelPJ{:});
    obj.csdFn.rC_from_mBodyTwist0_2_jointsAngVelPJ = casadi.Function('rC_from_mBodyTwist0_2_jointsAngVelPJ',{obj.csdSy.mBodyPosQuat_0},{obj.csdSy.rC_from_mBodyTwist0_2_jointsAngVelPJ});
    
    obj.csdSy.rC_from_jointsAngVelPJ_2_jointsAngVel0 = mystica.utils.getCasadiMatrixWithStructuredZeros(blkdiag(rC_from_jointsAngVelPJ_2_jointsAngVel0{:}),'casadiType','SX');
    obj.csdFn.rC_from_jointsAngVelPJ_2_jointsAngVel0 = casadi.Function('rC_from_jointsAngVelPJ_2_jointsAngVel0',{obj.csdSy.mBodyPosQuat_0},{obj.csdSy.rC_from_jointsAngVelPJ_2_jointsAngVel0});
    
    %% get_mBodyVelQuat0_from_mBodyTwist0
    
    mBodyTwist_0  = casadi.SX.sym('v',model.constants.mBodyTwist,1);    
    linkTwist_0   = casadi.SX.sym('linkTwist',model.constants.linkTwist,1);
    linkPosQuat_0 = casadi.SX.sym('linkPosQuat',model.constants.linkPosQuat,1);
    kBaum         = casadi.SX.sym('kBaum',1,1);
    
    linkVelQuat_0 = casadi.Function('f',{linkPosQuat_0,linkTwist_0},{[mystica.rbm.getPosGivenTwist(linkTwist_0);...
             mystica.rbm.getDotQuatGivenAngVel0(mystica.rbm.getQuatGivenPosQuat(linkPosQuat_0),mystica.rbm.getAngVelGivenTwist(linkTwist_0),kBaum)]});
    mBodyVelQuat0_csdFn_matrix = linkVelQuat_0.map(model.nLink);
    obj.csdFn.get_mBodyVelQuat0_from_mBodyTwist0 = casadi.Function('get_mBodyVelQuat_from_mBodyTwist',{obj.csdSy.mBodyPosQuat_0,mBodyTwist_0,kBaum},{reshape(mBodyVelQuat0_csdFn_matrix(...
        reshape(obj.csdSy.mBodyPosQuat_0,model.constants.linkPosQuat,model.nLink),...
        reshape(mBodyTwist_0,model.constants.linkTwist,model.nLink)),model.constants.mBodyPosQuat,1)});
    
end
