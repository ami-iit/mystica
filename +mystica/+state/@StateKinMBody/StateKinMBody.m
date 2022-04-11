classdef StateKinMBody < matlab.mixin.Copyable
    %STATEKINMBODY Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess=protected,GetAccess=public)
        mBodyPosQuat_0
        mBodyPosQuat_0_initial
        Jc
        nullEvaluator
        nullJc_mBodyTwist_0
        nullJc_jointsAngVel_PJ
        linIndRowJc
        referenceConversion
        linksState
        csdFn
        stgs
    end
    properties
        csdSy
    end

    methods
        function obj = StateKinMBody(input)
            arguments
                input.model          mystica.model.Model
                input.mBodyPosQuat_0 (:,1)
                input.stgsStateKinMBody
            end

            obj.stgs = input.stgsStateKinMBody;

            obj.csdSy.mBodyPosQuat_0 = casadi.SX.sym('x',input.model.constants.mBodyPosQuat,1);

            % links initialization
            obj.linksState{input.model.nLink} = {};
            for i = 1 : input.model.nLink
                obj.linksState{i} = mystica.state.LinkState('csdMBodyPosQuat',obj.csdSy.mBodyPosQuat_0,'indexesLinkPosQuat',input.model.linksAttributes{i}.selector.indexes_linkPosQuat_from_mBodyPosQuat);
            end
            obj.getJacobianConstraints(input.model)
            obj.getReferenceConversion(input.model)
            obj.nullEvaluator = mystica.utils.NullSpace(...
                'decompositionMethod'   ,obj.stgs.nullSpace.decompositionMethod,...
                'rankRevealingMethod'   ,obj.stgs.nullSpace.rankRevealingMethod,...
                'toleranceRankRevealing',obj.stgs.nullSpace.toleranceRankRevealing);
            obj.setMBodyPosQuat('mBodyPosQuat_0',input.mBodyPosQuat_0,'model',input.model)
            obj.mBodyPosQuat_0_initial = obj.mBodyPosQuat_0;
            input.model.constants.setNumberConstraints(size(obj.Jc,1)); % Note: @mystica.model.Model and @Constants are two handle classes! We are modifying model.constants in the main file
        end

        function clearProperties(obj)
            obj.csdFn = [];
            obj.csdSy = [];
            for i = 1 : length(obj.linksState)
                obj.linksState{i}.clearCasadi
            end
        end

        function intJcV = getIntJcV(obj)
            intJcV = full(obj.csdFn.intJcV(obj.mBodyPosQuat_0,obj.mBodyPosQuat_0_initial));
        end

        mBodyVelQuat    = get_mBodyVelQuat0_from_mBodyTwist0(obj,input);
        mBodyVelQuat    = get_mBodyVelQuat0_from_motorsAngVel(obj,input);
        mBodyTwist      = get_mBodyTwist0_from_motorsAngVel(obj,input)
        jointsAngVel_PJ = get_jointsAngVelPJ_from_motorsAngVel(obj,input);
        Zact = getZact(obj,input);
        setMBodyPosQuat(obj,input)
    end

    methods (Access=protected)
        getReferenceConversion(obj,model)
        getJacobianConstraints(obj,model)
        updateLinkState(obj,model)
    end

end
