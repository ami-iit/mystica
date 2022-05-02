classdef NoiseKinRel < handle
    %NOISEKINREL Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess=protected,GetAccess=public)
        stgsNoise
        timeIncrement
        kBaum
        regTermDampPInv
    end

    methods
        function [obj] = NoiseKinRel(input)
            arguments
                input.stgsNoise
                input.controller_dt
                input.kBaum
                input.regTermDampPInv
            end
            obj.stgsNoise       = input.stgsNoise;
            obj.timeIncrement   = input.controller_dt;
            obj.kBaum           = input.kBaum;
            obj.regTermDampPInv = input.regTermDampPInv;
        end

        function motorsAngVel = applyInputCompression(obj,input)
            arguments
                obj
                input.motorsAngVel
            end

            if obj.stgsNoise.inputCompression.bool
                mu = 0;
                saturationValue     = obj.stgsNoise.inputCompression.maxValue;
                probSaturationValue = obj.stgsNoise.inputCompression.probMaxValue/2;
                noise = abs(mystica.utils.createBoundedGaussianNoise(size(input.motorsAngVel),mu,saturationValue,probSaturationValue));
                motorsAngVel = input.motorsAngVel .* (1 - noise);
            else
                motorsAngVel = input.motorsAngVel;
            end

        end

        function stateKinMBody = createStateKinMBodyNoise(obj,input)
            arguments
                obj
                input.stateKinMBody
            end
            stateKinMBody = copy(input.stateKinMBody);
        end

        function stateKinMBody = applyEstimationError(obj,input)
            arguments
                obj
                input.stateKinMBody
                input.model
            end

            stateKinMBody = copy(input.stateKinMBody);

            if obj.stgsNoise.errorStateEstimation.bool
                mu = 0;
                saturationValue     = obj.stgsNoise.errorStateEstimation.maxValue;
                probSaturationValue = obj.stgsNoise.errorStateEstimation.probMaxValue;
                motorsAngVelNoise   = mystica.utils.createBoundedGaussianNoise([input.model.constants.motorsAngVel 1],mu,saturationValue,probSaturationValue);
                mBodyVelQuat_0 = stateKinMBody.get_mBodyVelQuat0_from_motorsAngVel('motorsAngVel',motorsAngVelNoise,...
                    'model',input.model,'kBaum',obj.kBaum,'regTermDampPInv',obj.regTermDampPInv);
                stateKinMBody.setMBodyPosQuat('model',input.model,...
                    'mBodyPosQuat_0',stateKinMBody.mBodyPosQuat_0 + mBodyVelQuat_0 * obj.timeIncrement)
            end

        end
    end
end
