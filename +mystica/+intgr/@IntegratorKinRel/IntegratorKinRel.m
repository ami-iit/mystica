classdef IntegratorKinRel < mystica.intgr.Integrator
    %INTEGRATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess=protected,GetAccess=public)
        motorsAngVel   %input
        mBodyPosQuat_0 %x
        mBodyVelQuat_0 %dxdt = f(input,x)
    end
    properties (SetAccess=immutable,GetAccess=public)
        dt
    end
    
    methods
        function obj = IntegratorKinRel(input)
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
                input.motorsAngVel
                input.stateKinMBody
                input.model
            end
            
            obj.ti = obj.ti + obj.dt;
            obj.tf = obj.tf + obj.dt;
            
            obj.motorsAngVel   = input.motorsAngVel;
            obj.mBodyPosQuat_0 = input.stateKinMBody.mBodyPosQuat_0;
            
            if obj.dxdtOpts.assumeConstant
                obj.mBodyVelQuat_0 = input.stateKinMBody.get_mBodyVelQuat0_from_motorsAngVel(...
                    'model',input.model,...
                    'motorsAngVel',obj.motorsAngVel,...
                    'kBaum',obj.dxdtParam.baumgarteIntegralCoefficient,...
                    'regTermDampPInv',obj.dxdtParam.regTermDampPInv);
                dxdt = @(t,x) obj.mBodyVelQuat_0;
            else
                dxdt = @(t,x) input.stateKinMBody.get_mBodyVelQuat0_from_motorsAngVel(...
                    'model',input.model,...
                    'motorsAngVel',obj.motorsAngVel,...
                    'kBaum',obj.dxdtParam.baumgarteIntegralCoefficient,...
                    'regTermDampPInv',obj.dxdtParam.regTermDampPInv,...
                    'mBodyPosQuat_0_warningOnlyIfNecessary',x);
            end
            
            xf = obj.integrate@mystica.intgr.Integrator('dxdt',dxdt,'xi',obj.mBodyPosQuat_0,'ti',obj.ti,'tf',obj.tf);
            
            if obj.dxdtOpts.assumeConstant == 0
                % because the stateKinMBody (handle class) is updated inside the method get_mBodyVelQuat0_from_motorsAngVel
                input.stateKinMBody.setMBodyPosQuat('model',input.model,'mBodyPosQuat_0',obj.mBodyPosQuat_0)
            end
            
        end
    end
    
end
