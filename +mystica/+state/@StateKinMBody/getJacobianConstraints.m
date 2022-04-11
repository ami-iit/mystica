function getJacobianConstraints(obj,model)
    %GETJACOBIANCONSTRAINTS Summary of this function goes here
    %   Detailed explanation goes here
    arguments
        obj   mystica.state.StateKinMBody
        model mystica.model.Model
    end
    
    Jc     = {};
    intJcV = {};
    cnstr_lin = {};
    cnstr_ang = {};
    
    mBodyPosQuat_0_initial = casadi.SX.sym('x_initial',model.constants.mBodyPosQuat,1);
    
    for i = transpose(model.indexesFixedLink(:))
        
        sel_twist_i = model.linksAttributes{i}.selector.matrix_linkTwist_from_mBodyTwist;
        linkPosQuat_0_initial = mBodyPosQuat_0_initial(model.linksAttributes{i}.selector.indexes_linkPosQuat_from_mBodyPosQuat);
        
        M = sel_twist_i;
        N = [linkPosQuat_0_initial(1:model.constants.linkPos);sparse(model.constants.linkAngVel,1)];
        
        Jc{       end+1} = M;
        intJcV{   end+1} = N;
        cnstr_lin{end+1} = [1 1 1 0 0 0]';
        cnstr_ang{end+1} = [0 0 0 1 1 1]';
    end
    
    for i = 1 : model.nJoint
        index = model.joints{i}.getJointConnectionDetails();
        
        tform_0_p  = obj.linksState{index.parent}.csdFn.tform_0_b(obj.csdSy.mBodyPosQuat_0);
        tform_0_c  = obj.linksState{index.child }.csdFn.tform_0_b(obj.csdSy.mBodyPosQuat_0);
        tform_p_pj = model.linksAttributes{index.parent}.tform_b_j{index.cPointParent};
        tform_c_cj = model.linksAttributes{index.child }.tform_b_j{index.cPointChild };
        
        if strcmp(...
                model.joints{i}.type,'spherical') || ...
                ...strcmp(model.joints{i}.type,'universal') || ...  %toBeDone! see https://github.com/ami-iit/element_morphing-cover-design/issues/86
                strcmp(model.joints{i}.type,'revolute')  || ...
                strcmp(model.joints{i}.type,'fixed')
            
            rotm_0_p    = mystica.rbm.getRotmGivenTform(tform_0_p);
            rotm_0_c    = mystica.rbm.getRotmGivenTform(tform_0_c);
            pos_p_pj    = mystica.rbm.getPosGivenTform(tform_p_pj);
            pos_c_cj    = mystica.rbm.getPosGivenTform(tform_c_cj);
            sel_twist_p = model.linksAttributes{index.parent}.selector.matrix_linkTwist_from_mBodyTwist;
            sel_twist_c = model.linksAttributes{index.child }.selector.matrix_linkTwist_from_mBodyTwist;
            
            M = transpose(rotm_0_p)*[eye(3) -mystica.utils.skew( rotm_0_p * pos_p_pj )]*sel_twist_p - ...
                transpose(rotm_0_p)*[eye(3) -mystica.utils.skew( rotm_0_c * pos_c_cj )]*sel_twist_c;
            
            N = transpose(rotm_0_p)*(mystica.rbm.getPosGivenTform(tform_0_p*tform_p_pj)-mystica.rbm.getPosGivenTform(tform_0_c*tform_c_cj));
            
            Jc{       end+1} = M;
            intJcV{   end+1} = N;
            cnstr_lin{end+1} = ones( size(M,1),1);
            cnstr_ang{end+1} = zeros(size(M,1),1);
            
        end
        if ...strcmp(model.joints{i}.type,'universal') || ...  %toBeDone! see https://github.com/ami-iit/element_morphing-cover-design/issues/86
                strcmp(model.joints{i}.type,'revolute')  || ...
                strcmp(model.joints{i}.type,'fixed')
            
            constrainedDirections = diag(not(model.joints{i}.axesRotation));
            constrainedDirections = constrainedDirections(sum(constrainedDirections,2)==1,:)*1;
            
            rotm_pj_0 = transpose(mystica.rbm.getRotmGivenTform(tform_0_p*tform_p_pj));
            sel_angVel_c = model.linksAttributes{index.child }.selector.matrix_linkAngVel_from_mBodyTwist;
            sel_angVel_p = model.linksAttributes{index.parent}.selector.matrix_linkAngVel_from_mBodyTwist;
            
            M = constrainedDirections*rotm_pj_0*(sel_angVel_c - sel_angVel_p);
            N = sparse(size(constrainedDirections,1),1);
            
            Jc{       end+1} = M;
            intJcV{   end+1} = N;
            cnstr_lin{end+1} = zeros(size(M,1),1);
            cnstr_ang{end+1} = ones( size(M,1),1);
            
        end
    end
    
    obj.csdSy.Jc = vertcat(Jc{:});
    obj.csdFn.Jc = casadi.Function('Jc',{obj.csdSy.mBodyPosQuat_0},{obj.csdSy.Jc});
    
    obj.csdSy.intJcV = vertcat(intJcV{:});
    obj.csdFn.intJcV = casadi.Function('intJcV',{obj.csdSy.mBodyPosQuat_0,mBodyPosQuat_0_initial},{obj.csdSy.intJcV});
    
    model.updateSelectorConstrainedDirections('indexes_ang',find(vertcat(cnstr_ang{:})),'indexes_lin',find(vertcat(cnstr_lin{:})))
    
end
