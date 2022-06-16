classdef IntegratorDynRel < mystica.intgr.Integrator
    %INTEGRATOR Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess=protected,GetAccess=public)
        motorsCurrent %input
        mBodyPosVel_0 %x
        mBodyVelAcc_0 %dxdt = f(input,x)
    end
    properties (SetAccess=immutable,GetAccess=public)
        dt
    end

    methods
        function obj = IntegratorDynRel(input)
            arguments
                input.stgsIntegrator struct;
                input.dt
            end
            obj@mystica.intgr.Integrator(...
                'dxdt'          ,[],...
                'xi'            ,[],...
                'ti'            ,-input.dt,...
                'tf'            ,0,...
                'stgsIntegrator',input.stgsIntegrator);
            obj.dt = input.dt;
        end

        function xf =  integrate(obj,input)
            arguments
                obj
                input.motorsCurrent
                input.stateDynMBody
                input.model
            end

            obj.ti = obj.ti + obj.dt;
            obj.tf = obj.tf + obj.dt;

            obj.motorsCurrent = input.motorsCurrent;
            obj.mBodyPosVel_0 = input.stateDynMBody.mBodyPosVel_0;

            switch obj.solverOpts.name
                case {'rk','cvodes'}
                    x    = casadi.MX.sym('x',input.model.constants.mBodyPosVel,1);
                    if obj.dxdtOpts.assumeConstant
                        dxdt = casadi.Function('dxdt',{x},{input.stateDynMBody.csdFn.mBodyVelAcc_0(obj.mBodyPosVel_0,obj.motorsCurrent)});
                    else
                        dxdt = casadi.Function('dxdt',{x},{input.stateDynMBody.csdFn.mBodyVelAcc_0(x,obj.motorsCurrent)});
                    end
                case {'ode45','ode23','ode113','ode78','ode89','ode15s','ode23s','ode23t','ode23tb','ode15i'} %https://it.mathworks.com/help/matlab/math/choose-an-ode-solver.html
                    if obj.dxdtOpts.assumeConstant
                        obj.mBodyVelAcc_0 = input.stateDynMBody.get_mBodyVelAcc0_from_motorsCurrent(...
                            'motorsCurrent',obj.motorsCurrent,...
                            'kBaum',obj.dxdtParam.baumgarteIntegralCoefficient,...
                            'model',input.model,...
                            'kFeedbackJcV',obj.dxdtParam.feedbackJacobianConstraintsV,...
                            'solverTechnique',obj.dxdtOpts.solverTechnique,...
                            't',t);
                        dxdt = @(t,x) obj.mBodyVelAcc_0;
                    else
                        dxdt = @(t,x) input.stateDynMBody.get_mBodyVelAcc0_from_motorsCurrent(...
                            'motorsCurrent',obj.motorsCurrent,...
                            'kBaum',obj.dxdtParam.baumgarteIntegralCoefficient,...
                            'model',input.model,...
                            'kFeedbackJcV',obj.dxdtParam.feedbackJacobianConstraintsV,...
                            'mBodyPosVel_0_warningOnlyIfNecessary',x,...
                            'solverTechnique',obj.dxdtOpts.solverTechnique,...
                            't',t);
                    end
                otherwise
                    error('not valid solver')
            end
            xf = obj.integrate@mystica.intgr.Integrator('dxdt',dxdt,'xi',obj.mBodyPosVel_0,'ti',obj.ti,'tf',obj.tf);

            if obj.dxdtOpts.assumeConstant == 0
                % because the stateDynMBody (handle class) is updated
                % inside the method get_mBodyVelAcc0_from_motorsCurrent
                input.stateDynMBody.setMBodyPosVel( 'model',input.model,'mBodyPosVel_0' ,obj.mBodyPosVel_0)
                input.stateDynMBody.setMBodyPosQuat('model',input.model,'mBodyPosQuat_0',obj.mBodyPosVel_0(input.model.selector.indexes_mBodyPosQuat_from_mBodyPosVel));
            end


        end
    end

end
