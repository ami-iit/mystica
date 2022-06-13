classdef LoggerDynRel < mystica.log.Logger
    %LOGGERKINABS Summary of this class goes here
    %   Detailed explanation goes here

    properties
        mBodyTwist_0
        motorsAngVel
        motorsCurrent
        jointsAngVel_PJ
        %mBodyAngVelStar_0
        mBodyWrenchExt_0
        mBodyWrenchGra_0
        mBodyWrenchCor_0
        mBodyWrenchFri_0
        mBodyWrenchInp_0
        mBodyWrenchJcF_0
        mBodyTwAcc_0
        jointsWrenchConstr_PJ
    end
    properties (SetAccess=immutable, GetAccess=protected)
        stgsIntegration
    end

    methods
        function obj = LoggerDynRel(input)
            %LOGGERKINABS Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                input.model
                input.stateDynMBody
                input.numberIterations
                input.stgsIntegrator
            end
            obj@mystica.log.Logger('model',input.model,'numberIterations',input.numberIterations)
            obj.mBodyTwist_0      = zeros(input.model.constants.mBodyTwist  ,obj.numberIterations);
            obj.motorsAngVel      = zeros(input.model.constants.motorsAngVel,obj.numberIterations);
            obj.motorsCurrent     = zeros(input.model.constants.motorsAngVel,obj.numberIterations);
            obj.jointsAngVel_PJ   = zeros(input.model.constants.jointsAngVel,obj.numberIterations);
            %obj.mBodyAngVelStar_0 = zeros(input.model.constants.mBodyAngVel ,obj.numberIterations);
            obj.mBodyWrenchExt_0      = zeros(input.model.constants.mBodyTwist,obj.numberIterations);
            obj.mBodyWrenchGra_0      = zeros(input.model.constants.mBodyTwist,obj.numberIterations);
            obj.mBodyWrenchCor_0      = zeros(input.model.constants.mBodyTwist,obj.numberIterations);
            obj.mBodyWrenchFri_0      = zeros(input.model.constants.mBodyTwist,obj.numberIterations);
            obj.mBodyWrenchInp_0      = zeros(input.model.constants.mBodyTwist,obj.numberIterations);
            obj.mBodyWrenchJcF_0      = zeros(input.model.constants.mBodyTwist,obj.numberIterations);
            obj.mBodyTwAcc_0          = zeros(input.model.constants.mBodyTwist,obj.numberIterations);
            obj.jointsWrenchConstr_PJ = zeros(input.model.constants.nConstraints,obj.numberIterations);

            obj.stgsIntegration.dxdtOpts  = input.stgsIntegrator.dxdtOpts;
            obj.stgsIntegration.dxdtParam = input.stgsIntegrator.dxdtParam;
        end

        function store(obj,input)
            arguments
                obj
                input.time
                input.model
                input.indexIteration
                input.stateDynMBody
                input.controller
                input.stgsDesiredShape
                input.motorsCurrent
            end

            obj.storeStateKinMBody('model',input.model,'controller',input.controller,'indexIteration',input.indexIteration,...
                'stateKinMBody',input.stateDynMBody,'stgsDesiredShape',input.stgsDesiredShape,'time',input.time);

            mBodyTwist   = input.stateDynMBody.mBodyPosVel_0(input.model.selector.indexes_mBodyTwist_from_mBodyPosVel);
            jointsAngVel = input.stateDynMBody.referenceConversion.from_mBodyTwist0_2_jointsAngVelPJ * mBodyTwist;

            [mBodyVelAcc_0,jointsWrenchConstr] = input.stateDynMBody.get_mBodyVelAcc0_from_motorsCurrent(...
                'motorsCurrent',input.motorsCurrent,...
                'kBaum',obj.stgsIntegration.dxdtParam.baumgarteIntegralCoefficient,...
                'model',input.model,...
                'kFeedbackJcV',obj.stgsIntegration.dxdtParam.feedbackJacobianConstraintsV,...
                'solverTechnique',obj.stgsIntegration.dxdtOpts.solverTechnique);

            obj.mBodyTwist_0(   :,obj.indexIteration) = mBodyTwist;
            obj.motorsAngVel(   :,obj.indexIteration) = jointsAngVel(input.model.selector.indexes_motorsAngVel_from_jointsAngVel);
            obj.motorsCurrent(  :,obj.indexIteration) = input.motorsCurrent;
            obj.jointsAngVel_PJ(:,obj.indexIteration) = jointsAngVel;

            % massMatrix mBodyTwAcc_0 = mBodyWrenchExt + mBodyWrenchJcF_0
            %
            % mBodyWrenchExt = - wrenchGravity - wrenchCoriolis + wrenchFriction + wrenchInput

            mBodyWrench_0 = input.stateDynMBody.csdFn.mBodyWrench_0('mBodyPosVel_0',input.stateDynMBody.mBodyPosVel_0,'motorsCurrent',input.motorsCurrent);

            obj.mBodyWrenchExt_0(:,obj.indexIteration) = full(mBodyWrench_0.ext);
            obj.mBodyWrenchGra_0(:,obj.indexIteration) = full(mBodyWrench_0.gravity);
            obj.mBodyWrenchCor_0(:,obj.indexIteration) = full(mBodyWrench_0.coriolis);
            obj.mBodyWrenchFri_0(:,obj.indexIteration) = full(mBodyWrench_0.friction);
            obj.mBodyWrenchInp_0(:,obj.indexIteration) = full(mBodyWrench_0.input);
            % mBodyWrenchJcF_0 = Jc' F
            obj.mBodyWrenchJcF_0(:,obj.indexIteration) = input.stateDynMBody.Jc'*jointsWrenchConstr;

            obj.mBodyTwAcc_0(:,obj.indexIteration) = mBodyVelAcc_0(input.model.selector.indexes_mBodyTwist_from_mBodyPosVel);
            obj.jointsWrenchConstr_PJ(:,obj.indexIteration) = jointsWrenchConstr;

        end

        function data = getDataSI(obj,input)
            arguments
                obj
                input.model
            end
            umc = input.model.unitMeas.converter;
            data = obj.getDataSI@mystica.log.Logger('model',input.model);
            data.mBodyTwist_0(input.model.selector.indexes_mBodyLinVel_from_mBodyTwist,:) = data.mBodyTwist_0(input.model.selector.indexes_mBodyLinVel_from_mBodyTwist,:)./(umc.length); %[m/s]
            data.mBodyTwAcc_0(input.model.selector.indexes_mBodyLinVel_from_mBodyTwist,:) = data.mBodyTwAcc_0(input.model.selector.indexes_mBodyLinVel_from_mBodyTwist,:)./(umc.length); %[m/s^2]
            data.mBodyWrenchExt_0 = convertWrench(data.mBodyWrenchExt_0,input.model,umc); %[N][Nm]
            data.mBodyWrenchGra_0 = convertWrench(data.mBodyWrenchGra_0,input.model,umc); %[N][Nm]
            data.mBodyWrenchCor_0 = convertWrench(data.mBodyWrenchCor_0,input.model,umc); %[N][Nm]
            data.mBodyWrenchFri_0 = convertWrench(data.mBodyWrenchFri_0,input.model,umc); %[N][Nm]
            data.mBodyWrenchInp_0 = convertWrench(data.mBodyWrenchInp_0,input.model,umc); %[N][Nm]
            data.mBodyWrenchJcF_0 = convertWrench(data.mBodyWrenchJcF_0,input.model,umc); %[N][Nm]

            data.jointsWrenchConstr_PJ(input.model.selector.indexes_constrainedLinVel_from_JcV,:) = data.jointsWrenchConstr_PJ(input.model.selector.indexes_constrainedLinVel_from_JcV,:)./(umc.mass*umc.length);    %[N]
            data.jointsWrenchConstr_PJ(input.model.selector.indexes_constrainedAngVel_from_JcV,:) = data.jointsWrenchConstr_PJ(input.model.selector.indexes_constrainedAngVel_from_JcV,:)./(umc.mass*umc.length.^2); %[Nm]

            function W = convertWrench(W,model,umc)
                W(model.selector.indexes_mBodyLinVel_from_mBodyTwist,:) = W(model.selector.indexes_mBodyLinVel_from_mBodyTwist,:)./(umc.mass*umc.length);    %[N]
                W(model.selector.indexes_mBodyAngVel_from_mBodyTwist,:) = W(model.selector.indexes_mBodyAngVel_from_mBodyTwist,:)./(umc.mass*umc.length.^2); %[Nm]
            end

        end

    end
end
