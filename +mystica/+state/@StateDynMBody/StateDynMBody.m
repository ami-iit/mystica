classdef StateDynMBody < mystica.state.StateKinMBody
    %STATEDYNMBODY Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess=protected,GetAccess=public)
        mBodyPosVel_0
    end
    properties (SetAccess=protected,GetAccess=protected)
        opti = []
        optiVar
    end

    methods
        function obj = StateDynMBody(input)
            arguments
                input.model
                input.mBodyPosQuat_0
                input.mBodyTwist_0
                input.stgsStateDynMBody
                input.stgsIntegrator
            end

            obj@mystica.state.StateKinMBody('model',input.model,...
                'mBodyPosQuat_0',input.mBodyPosQuat_0,...
                'stgsStateKinMBody',input.stgsStateDynMBody)

            % Definition Casadi Variables
            obj.csdSy.motorsCurrent  = casadi.SX.sym('I',input.model.constants.motorsAngVel,1);
            obj.csdSy.mBodyPosVel_0  = casadi.SX.sym('chi',input.model.constants.mBodyPosVel,1);

            obj.getDynamicQuantities(input.model,input.stgsIntegrator);

            obj.setMBodyPosVel('mBodyPosVel_0',[input.mBodyPosQuat_0;input.mBodyTwist_0],'model',input.model)
            obj.setMBodyPosQuat('mBodyPosQuat_0',obj.mBodyPosQuat_0,'model',input.model);

        end
        [mBodyVelAcc_0,varargout] = get_mBodyVelAcc0_from_motorsCurrent(obj,input,stgs)

        function setMBodyPosVel(obj,input)
            arguments
                obj                 mystica.state.StateDynMBody
                input.mBodyPosVel_0 (:,1)
                input.model         mystica.model.Model
            end

            obj.mBodyPosVel_0  = input.mBodyPosVel_0;

            switch 'reduceComputationOfQuantities_mBodyPosQuat_Depedent'
                case 'computeQuantities_mBodyPosQuat_Depedent'
                    obj.setMBodyPosQuat('model',input.model,'mBodyPosQuat_0',obj.mBodyPosVel_0(input.model.selector.indexes_mBodyPosQuat_from_mBodyPosVel))
                case 'reduceComputationOfQuantities_mBodyPosQuat_Depedent'
                    % remove obj.setMBodyPosQuat to reduce computational time (see https://github.com/ami-iit/element_morphing-cover-design/commit/804e87f9b8d9dc144e09a0fc51e1e79b38e38a3a)
                    obj.mBodyPosQuat_0 = obj.mBodyPosVel_0(input.model.selector.indexes_mBodyPosQuat_from_mBodyPosVel);
                    obj.Jc                                                    = [];
                    obj.referenceConversion.from_mBodyTwist0_2_jointsAngVelPJ = [];
                    obj.nullJc_mBodyTwist_0                                   = [];
                    obj.nullJc_jointsAngVel_PJ                                = [];
                    obj.linIndRowJc                                           = [];
            end

        end

    end
    methods (Access=protected)
        getDynamicQuantities(obj,model,stgsIntegrator)
    end
end
