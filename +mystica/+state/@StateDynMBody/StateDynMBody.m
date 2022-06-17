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

        end
        [mBodyVelAcc_0,varargout] = get_mBodyVelAcc0_from_motorsCurrent(obj,input,stgs)

        function setMBodyPosVel(obj,input)
            arguments
                obj                 mystica.state.StateDynMBody
                input.mBodyPosVel_0 (:,1)
                input.model         mystica.model.Model
                input.method = 'full'
            end

            obj.mBodyPosVel_0  = input.mBodyPosVel_0;
            obj.setMBodyPosQuat('model',input.model,'mBodyPosQuat_0',obj.mBodyPosVel_0(input.model.selector.indexes_mBodyPosQuat_from_mBodyPosVel),'method',input.method)

        end

    end
    methods (Access=protected)
        getDynamicQuantities(obj,model,stgsIntegrator)
    end
end
