classdef NoiseDynRel < handle
    %NOISEKINREL Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess=protected,GetAccess=public)
        stgsNoise
        timeIncrement
        kBaum
        regTermDampPInv
    end

    methods
        function [obj] = NoiseDynRel(input)
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

        function motorsCurrent = applyInputCompression(obj,input)
            arguments
                obj
                input.motorsCurrent
            end

            if obj.stgsNoise.inputCompression.bool
                mu = 0;
                saturationValue     = obj.stgsNoise.inputCompression.maxValue;
                probSaturationValue = obj.stgsNoise.inputCompression.probMaxValue/2;
                noise = abs(mystica.utils.createBoundedGaussianNoise(size(input.motorsCurrent),mu,saturationValue,probSaturationValue));
                motorsCurrent = input.motorsCurrent .* (1 - noise);
            else
                motorsCurrent = input.motorsCurrent;
            end

        end

        function stateDynMBody = createStateDynMBodyNoise(obj,input)
            arguments
                obj
                input.stateDynMBody
            end
            stateDynMBody = copy(input.stateDynMBody);
        end

        function stateDynMBody = applyEstimationError(obj,input)
            arguments
                obj
                input.stateDynMBody
                input.model
            end

            stateDynMBody = copy(input.stateDynMBody);

            if obj.stgsNoise.errorStateEstimation.bool
                mu = 0;
                saturationValue     = obj.stgsNoise.errorStateEstimation.maxValue;
                probSaturationValue = obj.stgsNoise.errorStateEstimation.probMaxValue;
                motorsAngVelNoise   = mystica.utils.createBoundedGaussianNoise([input.model.constants.motorsAngVel 1],mu,saturationValue,probSaturationValue);
                mBodyVelQuat_0 = stateDynMBody.get_mBodyVelQuat0_from_motorsAngVel('motorsAngVel',motorsAngVelNoise,...
                    'model',input.model,'kBaum',obj.kBaum,'regTermDampPInv',obj.regTermDampPInv);
                mBodyPosQuat_0 = stateDynMBody.mBodyPosQuat_0 + mBodyVelQuat_0 * obj.timeIncrement;
                mBodyPosVel_0 = stateDynMBody.mBodyPosVel_0;
                mBodyPosVel_0(input.model.selector.indexes_mBodyPosQuat_from_mBodyPosVel) = mBodyPosQuat_0;
                stateDynMBody.setMBodyPosVel( 'model',input.model,'mBodyPosVel_0' ,mBodyPosVel_0)

            end

        end
    end
end
