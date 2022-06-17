function [mBodyVelAcc_0,varargout] = get_mBodyVelAcc0_from_motorsCurrent(obj,input,stgs)
    arguments
        obj
        input.motorsCurrent (:,1)
        input.model mystica.model.Model
        input.kBaum (1,1)
        input.mBodyPosVel_0_warningOnlyIfNecessary  (:,1)
        input.kFeedbackJcV
        input.t = inf
        stgs.solverTechnique char {mustBeMember(stgs.solverTechnique,{'inv','opt'})}
    end

    %---------------------------------------------------------------------%
    % Problem:
    % { M dV = W + Jc'f
    % { Jc dV + dJc V = 0
    %
    % - Compute dV and f
    % - Compute `mBodyVelAcc0` and `jointsWrenchConstr_PJ`
    %---------------------------------------------------------------------%

    if isfield(input,'mBodyPosVel_0_warningOnlyIfNecessary')
        obj.setMBodyPosVel('mBodyPosVel_0',input.mBodyPosVel_0_warningOnlyIfNecessary,'model',input.model)
    end

    switch stgs.solverTechnique
        case 'inv'

            %-----------------------------------------------------------------%
            % Inverse
            %-----------------------------------------------------------------%

            obj.setMBodyPosQuat('mBodyPosQuat_0',obj.mBodyPosQuat_0,'model',input.model);

            kpFeedbackJcV = input.kFeedbackJcV(1);
            kiFeedbackJcV = input.kFeedbackJcV(2);

            mBodyTwist_0     = obj.mBodyPosVel_0(input.model.selector.indexes_mBodyTwist_from_mBodyPosVel);
            mBodyVelQuat_0   = sparse(mystica_stateKin('get_mBodyVelQuat0_from_mBodyTwist0',obj.mBodyPosQuat_0,mBodyTwist_0,input.kBaum));

            Jc        = obj.Jc( obj.linIndRowJc,:);
            dJc       = sparse(obj.csdFn.dJc(obj.mBodyPosVel_0));
            dJc       = dJc(obj.linIndRowJc,:);
            intJcV    = obj.getIntJcV;
            intJcV    = intJcV( obj.linIndRowJc,:);
            V         = mBodyTwist_0;
            M         = sparse(obj.csdFn.massMatrix(obj.mBodyPosQuat_0));                             % massMatrix            -> M
            W         = sparse(obj.csdFn.mBodyWrenchExt_0(obj.mBodyPosVel_0,input.motorsCurrent));    % mBodyWrenchExt_0      -> W
            invMJcT   = mystica.utils.scaling(M)*((M*mystica.utils.scaling(M))\Jc');
            invMW     = mystica.utils.scaling(M)*((M*mystica.utils.scaling(M))\W);
            JcInvMJcT = Jc*invMJcT;
            % Jc dV + dJc V = fdbkJc = - kp Jc V - ki intJcV
            fdbkJc    = -kpFeedbackJcV*Jc*V -kiFeedbackJcV*intJcV;
            f         = mystica.utils.scaling(JcInvMJcT)*((JcInvMJcT*mystica.utils.scaling(JcInvMJcT))\(-Jc*invMW-dJc*V+fdbkJc)); % jointsWrenchConstr_PJ -> f
            dV        = invMJcT*f + invMW; % mBodyTwAcc_0 -> dV

            jointsWrenchConstr_PJ                    = zeros(input.model.constants.nConstraints,1);
            jointsWrenchConstr_PJ(obj.linIndRowJc,1) = f;
            mBodyTwAcc_0                             = dV;

            mBodyVelAcc_0 = [mBodyVelQuat_0 ; mBodyTwAcc_0];

        case 'opt'

            %-----------------------------------------------------------------%
            % Optimization (CasADi)
            %-----------------------------------------------------------------%

            if 0
                [mBodyVelAcc_0,jointsWrenchConstr_PJ] = obj.csdFn.mBodyVelAcc_0(obj.mBodyPosVel_0,input.motorsCurrent);
            else
                obj.opti.set_value(obj.optiVar.mBodyPosVel,obj.mBodyPosVel_0);
                obj.opti.set_value(obj.optiVar.motorsCurrent,input.motorsCurrent);
                sol = obj.opti.solve();
                X = sol.value(obj.optiVar.X);

                mBodyTwist_0     = obj.mBodyPosVel_0(input.model.selector.indexes_mBodyTwist_from_mBodyPosVel);
                mBodyVelQuat_0   = sparse(mystica_stateKin('get_mBodyVelQuat0_from_mBodyTwist0',obj.mBodyPosQuat_0,mBodyTwist_0,input.kBaum));

                mBodyTwAcc_0          = X(1:input.model.constants.mBodyTwist,1);
                jointsWrenchConstr_PJ = X(input.model.constants.mBodyTwist+1:end,1);
                mBodyVelAcc_0 = [mBodyVelQuat_0 ; mBodyTwAcc_0];

                obj.setMBodyPosQuat('mBodyPosQuat_0',obj.mBodyPosQuat_0,'model',input.model);
            end

            mBodyVelAcc_0         = full(mBodyVelAcc_0);
            jointsWrenchConstr_PJ = full(jointsWrenchConstr_PJ);

    end

    %---------------------------------------------------------------------%
    % Output

    if nargout > 1
        varargout{1} = jointsWrenchConstr_PJ;
    end
    if nargout > 2
        varargout{2} = mBodyWrenchExt_0;
    end

end
