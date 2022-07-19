function getDynamicQuantities(obj,model,stgsIntegrator,stgsModel)

    %% Compute Casadi Symbolic Variables

    obj.csdSy.mBodyPosQuat_0 = obj.csdSy.mBodyPosVel_0(model.selector.indexes_mBodyPosQuat_from_mBodyPosVel);
    obj.csdSy.mBodyTwist_0   = obj.csdSy.mBodyPosVel_0(model.selector.indexes_mBodyTwist_from_mBodyPosVel);
    obj.csdSy.mBodyVelQuat_0 = obj.csdFn.get_mBodyVelQuat0_from_mBodyTwist0(obj.csdSy.mBodyPosQuat_0,obj.csdSy.mBodyTwist_0,stgsIntegrator.dxdtParam.baumgarteIntegralCoefficient);

    obj.csdSy.Jc                                     = obj.csdFn.Jc(obj.csdSy.mBodyPosQuat_0);
    obj.csdSy.rC_from_mBodyTwist0_2_jointsAngVelPJ   = obj.csdFn.rC_from_mBodyTwist0_2_jointsAngVelPJ(  obj.csdSy.mBodyPosQuat_0);
    obj.csdSy.rC_from_jointsAngVelPJ_2_jointsAngVel0 = obj.csdFn.rC_from_jointsAngVelPJ_2_jointsAngVel0(obj.csdSy.mBodyPosQuat_0);

    obj.csdSy.dJc = reshape(jacobian(obj.csdSy.Jc,obj.csdSy.mBodyPosQuat_0)*obj.csdSy.mBodyVelQuat_0,size(obj.csdSy.Jc));

    jointsAngVelPJ = obj.csdSy.rC_from_mBodyTwist0_2_jointsAngVelPJ * obj.csdSy.mBodyTwist_0;

    %% Joint | frictionActAngVel, frictionPasAngVel, Ki, Kdw

    Ki{ model.nJoint}               = [];
    Kdw{model.nJoint}               = [];
    frictionActAngVel{model.nJoint} = [];
    frictionPasAngVel{model.nJoint} = [];

    for j = 1 : model.nJoint
        jAngVel_pj_cj = jointsAngVelPJ(model.joints{j}.selector.indexes_jAngVel_from_jointsAngVel );
        boolJointActuated =  model.joints{j}.axesActuated;
        boolJointPassive  = ~model.joints{j}.axesActuated & model.joints{j}.axesRotation;
        if any(boolJointActuated)
            indexTemp = find(boolJointActuated);
            actAxesAngVelPJ = jAngVel_pj_cj(indexTemp);
            eff   = model.joints{j}.transmission.efficiency(indexTemp,indexTemp);
            gamma = model.joints{j}.transmission.gearRatio(indexTemp,indexTemp);
            Kt    = model.joints{j}.coefficients.motorTorque(indexTemp,indexTemp);
            Kc    = model.joints{j}.coefficients.coulombFriction(indexTemp,indexTemp);
            Kv    = model.joints{j}.coefficients.viscousFriction(indexTemp,indexTemp);
            Jm    = model.joints{j}.inertiaTensMotorRot_j_g(indexTemp,indexTemp);
            % friction actuated axis of rotation
            viscousFriction = - Kv * gamma.^2 * eff * actAxesAngVelPJ;
            coulombFriction = - Kc * gamma * eff * mystica.utils.smoothSign(actAxesAngVelPJ,'method',stgsModel.friction.coulomb.smoothSign.method,'settlingTime',stgsModel.friction.coulomb.smoothSign.settlingTime,'toleranceBands',stgsModel.friction.coulomb.smoothSign.toleranceBands);
            frictionActAngVel{j} = viscousFriction*stgsModel.friction.viscous.apply + coulombFriction*stgsModel.friction.coulomb.apply;
            Ki{j}  = eff * gamma * Kt;
            Kdw{j} = Jm * gamma.^2 * eff;
        end
        if any(boolJointPassive)
            indexTemp = find(boolJointPassive);
            pasAxesAngVelPJ = jAngVel_pj_cj(indexTemp);
            Kc    = model.joints{j}.coefficients.coulombFriction(indexTemp,indexTemp);
            Kv    = model.joints{j}.coefficients.viscousFriction(indexTemp,indexTemp);
            % friction passive axis of rotation (actuated joints + passive joints)
            viscousFriction = - Kv * pasAxesAngVelPJ;
            coulombFriction = - Kc * mystica.utils.smoothSign(pasAxesAngVelPJ,'method',stgsModel.friction.coulomb.smoothSign.method,'settlingTime',stgsModel.friction.coulomb.smoothSign.settlingTime,'toleranceBands',stgsModel.friction.coulomb.smoothSign.toleranceBands);
            frictionPasAngVel{j} = viscousFriction*stgsModel.friction.viscous.apply + coulombFriction*stgsModel.friction.coulomb.apply;
        end
    end

    Ki                = sparse(blkdiag(Ki{:}));
    Kdw               = sparse(blkdiag(Kdw{:}));
    frictionActAngVel = vertcat(frictionActAngVel{:});
    frictionPasAngVel = vertcat(frictionPasAngVel{:});

    %% Link | massMatrixWithoutMotorInertia, wrenchGravity, wrenchCoriolis, B

    massMatrixWithoutMotorInertia{model.nLink} = [];
    wrenchGravity{ model.nLink} = [];
    wrenchCoriolis{model.nLink} = [];
    B{2*model.nLink} = [];
    B(:)={sparse(model.constants.linkPos,model.constants.jointsAngVel)};

    for i = 1 : model.nLink
        mass            = model.linksAttributes{i}.mass;
        rotm_0_b        = mystica.rbm.getRotmGivenTform(obj.get_link_tform_b('iLink',i,'model',model,'mBodyPosQuat_0',obj.csdSy.mBodyPosQuat_0));
        pos_b_g         = mystica.rbm.getPosGivenTform(model.linksAttributes{i}.tform_b_g);
        inertiaTens_0_b = rotm_0_b * model.linksAttributes{i}.inertiaTens_b_b * transpose(rotm_0_b);
        sel_twist       = model.linksAttributes{i}.selector.matrix_linkTwist_from_mBodyTwist;
        angVel_0_b      = obj.csdSy.mBodyTwist_0(model.linksAttributes{i}.selector.indexes_linkAngVel_from_mBodyTwist);

        massMatrixWithoutMotorInertia{i} = ...
            [eye(model.constants.linkPos)*mass, -mass*mystica.utils.skew(rotm_0_b*pos_b_g);...
            mass*mystica.utils.skew(rotm_0_b*pos_b_g),               inertiaTens_0_b]*sel_twist;

        wrenchGravity{ i} = -mass*[model.constants.gravity ; mystica.utils.skew(rotm_0_b*pos_b_g)*model.constants.gravity];
        wrenchCoriolis{i} = [mass*mystica.utils.skew(angVel_0_b)^2*rotm_0_b*pos_b_g; mystica.utils.skew(angVel_0_b)*inertiaTens_0_b*angVel_0_b];

        for j = model.linksAttributes{i}.joints(:)'
            signF  = - (model.joints{j}.linkParent == i) + (model.joints{j}.linkChild == i);
            B{2*i} = B{2*i} + signF*model.joints{j}.selector.matrix_jAngVel_from_jointsAngVel;
        end

    end

    massMatrixWithoutMotorInertia = vertcat(massMatrixWithoutMotorInertia{:});
    wrenchGravity  = vertcat(wrenchGravity{ :});
    wrenchCoriolis = vertcat(wrenchCoriolis{:});
    B = vertcat(B{:});

    %% massMatrix, massMatrixMotorInertia, dJc

    rC_w_V   = obj.csdSy.rC_from_mBodyTwist0_2_jointsAngVelPJ;
    rC_0_PJ  = obj.csdSy.rC_from_jointsAngVelPJ_2_jointsAngVel0;
    sel_wAct = model.selector.matrix_motorsAngVel_from_jointsAngVel;
    sel_wPas = model.selector.matrix_passiveAngVel_from_jointsAngVel;

    massMatrixMotorInertia = B * rC_0_PJ * sel_wAct' * Kdw * sel_wAct * rC_w_V;

    massMatrix = massMatrixWithoutMotorInertia + massMatrixMotorInertia;

    obj.csdFn.dJc                    = casadi.Function('dJc',{obj.csdSy.mBodyPosVel_0 },{obj.csdSy.dJc});
    obj.csdFn.massMatrix             = casadi.Function('massMatrix',{  obj.csdSy.mBodyPosQuat_0},{massMatrix});
    obj.csdFn.massMatrixMotorInertia = casadi.Function('massMatrixMotorInertia',{obj.csdSy.mBodyPosQuat_0},{massMatrixMotorInertia});

    %% mBodyWrenchExt_0, mBodyWrenchCor_0, mBodyWrenchGra_0, mBodyWrenchFri_0, mBodyWrenchInp_0

    wrenchCurrent  = B * rC_0_PJ * sel_wAct' * Ki * obj.csdSy.motorsCurrent;

    wrenchFriction = B * rC_0_PJ * sel_wAct' * frictionActAngVel;
    if ~isempty(frictionPasAngVel)
        wrenchFriction = wrenchFriction + B * rC_0_PJ * sel_wPas' * frictionPasAngVel;
    end

    % M*dV + wrenchCoriolis + wrenchGravity = B * motorCurrent + friction + Jc'f
    %
    % mBodyWrenchExt = B * motorCurrent + friction - wrenchCoriolis - wrenchGravity
    %
    % M*dV = Jc'f + mBodyWrenchExt

    wrenchTotal = -wrenchGravity-wrenchCoriolis+wrenchFriction+wrenchCurrent;

    obj.csdFn.mBodyWrenchExt_0 = casadi.Function('mBodyWrenchExt_0',{obj.csdSy.mBodyPosVel_0,obj.csdSy.motorsCurrent},{wrenchTotal},{'mBodyPosVel_0','motorsCurrent'},{'mBodyWrenchExt_0'});
    obj.csdFn.mBodyWrenchGra_0 = casadi.Function('mBodyWrenchGra_0',{obj.csdSy.mBodyPosQuat_0},{wrenchGravity},{'mBodyPosQuat_0'},{'mBodyWrenchGra_0'});
    obj.csdFn.mBodyWrenchCor_0 = casadi.Function('mBodyWrenchCor_0',{obj.csdSy.mBodyPosVel_0},{wrenchCoriolis},{'mBodyPosVel_0'},{'mBodyWrenchCor_0'});
    obj.csdFn.mBodyWrenchFri_0 = casadi.Function('mBodyWrenchFri_0',{obj.csdSy.mBodyPosVel_0},{wrenchFriction},{'mBodyPosVel_0'},{'mBodyWrenchFri_0'});
    obj.csdFn.mBodyWrenchInp_0 = casadi.Function('mBodyWrenchInp_0',{obj.csdSy.mBodyPosQuat_0,obj.csdSy.motorsCurrent},{wrenchCurrent},{'mBodyPosQuat_0','motorsCurrent'},{'mBodyWrenchInp_0'});

    obj.csdFn.mBodyWrench_0    = casadi.Function('mBodyWrench_0',...
        {obj.csdSy.mBodyPosVel_0,obj.csdSy.motorsCurrent},...                       %input
        {wrenchTotal,wrenchCoriolis,wrenchGravity,wrenchFriction,wrenchCurrent},... %output
        {'mBodyPosVel_0','motorsCurrent'},...                                       %label input
        {'ext','coriolis','gravity','friction','input'});                           %label output

    obj.csdFn.from_motorsCurrent_2_mBodyWrenchInp0 = casadi.Function('from_motorsCurrent_2_mBodyWrenchInp0',{obj.csdSy.mBodyPosQuat_0},{B * rC_0_PJ * sel_wAct' * Ki});

    %% mBodyVelAcc0

    kpFeedbackJcV = stgsIntegrator.dxdtParam.feedbackJacobianConstraintsV(1);
    kiFeedbackJcV = stgsIntegrator.dxdtParam.feedbackJacobianConstraintsV(2);

    switch stgsIntegrator.dxdtOpts.optOpts.name
        case 'qpoases'
            opti = casadi.Opti('conic');
            p_opts = struct('expand',true,'error_on_fail',false,'printLevel','none','enableFullLITests',true,'enableCholeskyRefactorisation',1,'numRefinementSteps',3);
            s_opts = struct();
        case 'osqp'
            opti = casadi.Opti('conic');
            p_opts = struct('expand',true,'error_on_fail',false);
            s_opts = struct('adaptive_rho',1);
        case 'ipopt'
            opti = casadi.Opti();
            p_opts = struct('expand',true,'error_on_fail',false,'print_time',false);
            s_opts = struct('print_level',0);
        otherwise
            error('solver not valid')
    end
    opti.solver(stgsIntegrator.dxdtOpts.optOpts.name,p_opts,s_opts);

    % scaling opti variable
    k_dV = ones(model.constants.mBodyTwist,1);
    for i = 1 : model.nLink
        k_dV(model.linksAttributes{i}.selector.indexes_linkLinVel_from_mBodyTwist) = (model.linksAttributes{i}.linkDimension);
        k_dV(model.linksAttributes{i}.selector.indexes_linkAngVel_from_mBodyTwist) = 1;
    end
    k_f = ones(model.constants.nConstraints,1);
    k_f(model.selector.indexes_constrainedAngVel_from_JcV) = (model.linksAttributes{1}.mass*model.linksAttributes{1}.linkDimension^2);
    k_f(model.selector.indexes_constrainedLinVel_from_JcV) = (model.linksAttributes{1}.mass*model.linksAttributes{1}.linkDimension);
    k_f  = diag(k_f);
    k_dV = diag(k_dV);

    % Variable definitions
    dV            = k_dV*opti.variable(model.constants.mBodyTwist ,1); % mBodyTwAcc_0 -> dV
    f             = k_f*opti.variable(model.constants.nConstraints,1);  % jointsWrenchConstr_pj -> f
    x             = [dV;f];
    mBodyPosVel   = opti.parameter(model.constants.mBodyPosVel ,1);
    motorsCurrent = opti.parameter(model.constants.motorsAngVel,1);
    %
    mBodyPosQuat = mBodyPosVel(model.selector.indexes_mBodyPosQuat_from_mBodyPosVel);
    V      = mBodyPosVel(model.selector.indexes_mBodyTwist_from_mBodyPosVel);         % mBodyTwist_0          -> V
    M      = obj.csdFn.massMatrix(mBodyPosQuat);                                      % massMatrix            -> M
    W      = obj.csdFn.mBodyWrenchExt_0(mBodyPosVel,motorsCurrent);                   % mBodyWrenchExt_0      -> W
    Jc     = obj.csdFn.Jc(mBodyPosQuat);
    dJc    = obj.csdFn.dJc(mBodyPosVel);
    intJcV = obj.csdFn.intJcV(mBodyPosQuat,obj.mBodyPosQuat_0_initial);
    % Cost Function & Constraint
    E = (Jc*dV+dJc*V)+(kpFeedbackJcV*Jc*V)+(kiFeedbackJcV*intJcV);
    D = M*dV-W-Jc'*f;
    opti.minimize(f'*f+dV'*dV);
    opti.subject_to(D==0);
    opti.subject_to(((Jc*V)'*(Jc*dV+dJc*V)) < 0);
    optFun = opti.to_function('mBodyVelAcc',{mBodyPosVel,motorsCurrent},{x});
    %
    X                     = optFun(mBodyPosVel,motorsCurrent);
    mBodyVelQuat          = obj.csdFn.get_mBodyVelQuat0_from_mBodyTwist0(mBodyPosQuat,V,stgsIntegrator.dxdtParam.baumgarteIntegralCoefficient);
    mBodyTwAcc_0          = X(1:model.constants.mBodyTwist,1);
    jointsWrenchConstr_PJ = X(model.constants.mBodyTwist+1:end,1);
    mBodyVelAcc_0 = [mBodyVelQuat ; mBodyTwAcc_0];
    %
    obj.csdFn.mBodyVelAcc_0 = casadi.Function('mBodyVelAcc_0',...
        {mBodyPosVel,motorsCurrent},...
        {mBodyVelAcc_0,jointsWrenchConstr_PJ},...
        {'mBodyPosVel','motorsCurrent'},...
        {'mBodyVelAcc_0','jointsWrenchConstr_PJ'});

    % alternative for computing [dV;f]
    obj.opti                  = opti;
    obj.optiVar.mBodyPosVel   = mBodyPosVel;
    obj.optiVar.motorsCurrent = motorsCurrent;
    obj.optiVar.X             = X;

end
