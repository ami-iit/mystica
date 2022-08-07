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
        linkPosQuat_0_initial = mBodyPosQuat_0_initial(  model.linksAttributes{i}.selector.indexes_linkPosQuat_from_mBodyPosQuat);
        linkPosQuat_0         = obj.csdSy.mBodyPosQuat_0(model.linksAttributes{i}.selector.indexes_linkPosQuat_from_mBodyPosQuat);

        quat_bi_0 = mystica.rbm.invQuat(mystica.rbm.getQuatGivenPosQuat(linkPosQuat_0_initial));
        quat_0_b  = mystica.rbm.getQuatGivenPosQuat(linkPosQuat_0);
        pos_0_bi  = mystica.rbm.getPosGivenPosQuat( linkPosQuat_0_initial);
        pos_0_b   = mystica.rbm.getPosGivenPosQuat( linkPosQuat_0);

        M = sel_twist_i;
        N = [pos_0_b-pos_0_bi;mystica.rbm.logQuat(mystica.rbm.multiplyQuat(quat_0_b,quat_bi_0),'selectQuat','minDistance')];

        Jc{       end+1} = M;
        intJcV{   end+1} = N;
        cnstr_lin{end+1} = [1 1 1 0 0 0]';
        cnstr_ang{end+1} = [0 0 0 1 1 1]';
    end

    for i = 1 : model.nJoint
        index = model.joints{i}.getJointConnectionDetails();

        tform_0_p  = obj.get_link_tform_b('iLink',index.parent,'model',model,'mBodyPosQuat_0',obj.csdSy.mBodyPosQuat_0);
        tform_0_c  = obj.get_link_tform_b('iLink',index.child ,'model',model,'mBodyPosQuat_0',obj.csdSy.mBodyPosQuat_0);
        tform_p_pj = model.linksAttributes{index.parent}.tform_b_j{index.cPointParent};
        tform_c_cj = model.linksAttributes{index.child }.tform_b_j{index.cPointChild };
        tform_0_pj = tform_0_p*tform_p_pj;
        tform_0_cj = tform_0_c*tform_c_cj;

        quat_0_p = mystica.rbm.getQuatGivenPosQuat(obj.csdSy.mBodyPosQuat_0(model.linksAttributes{index.parent}.selector.indexes_linkPosQuat_from_mBodyPosQuat));
        quat_0_c = mystica.rbm.getQuatGivenPosQuat(obj.csdSy.mBodyPosQuat_0(model.linksAttributes{index.child }.selector.indexes_linkPosQuat_from_mBodyPosQuat));

        quat_pj_0    = mystica.rbm.invQuat(mystica.rbm.multiplyQuat(quat_0_p,mystica.rbm.getQuatGivenTform(tform_p_pj)));
        quat_0_cj    = mystica.rbm.multiplyQuat(quat_0_c,mystica.rbm.getQuatGivenTform(tform_c_cj));
        quat_pj_cj   = mystica.rbm.multiplyQuat(quat_pj_0,quat_0_cj);

        quat_cj0_pj0 = mystica.rbm.invQuat(mystica.rbm.getQuatGivenTform(model.joints{i}.tform_pj0_cj0));

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

            N = transpose(rotm_0_p)*(mystica.rbm.getPosGivenTform(tform_0_pj)-mystica.rbm.getPosGivenTform(tform_0_cj));

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
            N = constrainedDirections*mystica.rbm.logQuat(mystica.rbm.multiplyQuat(quat_pj_cj,quat_cj0_pj0),'selectQuat','minDistance');

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

    model.appendSelectorConstrainedDirections('indexes_ang',find(vertcat(cnstr_ang{:})),'indexes_lin',find(vertcat(cnstr_lin{:})))

end
