classdef LoggerKinRel < mystica.log.Logger
    %LOGGERKINABS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mBodyTwist_0
        motorsAngVel
        motorsAngVelNoise
        jointsAngVel_PJ
        mBodyAngVelStar_0
    end
    
    methods
        function obj = LoggerKinRel(input)
            %LOGGERKINABS Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                input.model
                input.numberIterations
            end
            obj@mystica.log.Logger('model',input.model,'numberIterations',input.numberIterations)
            obj.mBodyAngVelStar_0 = zeros(input.model.constants.mBodyAngVel ,obj.numberIterations);
            obj.motorsAngVel      = zeros(input.model.constants.motorsAngVel,obj.numberIterations);
            obj.motorsAngVelNoise = zeros(input.model.constants.motorsAngVel,obj.numberIterations);
            obj.jointsAngVel_PJ   = zeros(input.model.constants.jointsAngVel,obj.numberIterations);
            obj.mBodyTwist_0      = zeros(input.model.constants.mBodyTwist  ,obj.numberIterations);
        end
        
        function store(obj,input)
            arguments
                obj
                input.time
                input.model
                input.indexIteration
                input.stateKinMBody
                input.stateKinMBodyNoise
                input.controller
                input.stgsDesiredShape
                input.motorsAngVel
                input.motorsAngVelNoise
                input.regTermDampPInv
            end
            
            
            obj.storeStateKinMBody('model',input.model,'controller',input.controller,'indexIteration',input.indexIteration,...
                'stateKinMBody',input.stateKinMBody,'stgsDesiredShape',input.stgsDesiredShape,'time',input.time);
            
            obj.motorsAngVel(:,obj.indexIteration)    = input.motorsAngVel;
            obj.jointsAngVel_PJ(:,obj.indexIteration) = input.stateKinMBody.get_jointsAngVelPJ_from_motorsAngVel(...
                'model',input.model,'motorsAngVel',input.motorsAngVel,'regTermDampPInv',input.regTermDampPInv);
            obj.mBodyTwist_0(:,obj.indexIteration)    = input.stateKinMBody.get_mBodyTwist0_from_motorsAngVel(...
                'model',input.model,'motorsAngVel',input.motorsAngVel,'regTermDampPInv',input.regTermDampPInv);
            
            obj.mBodyAngVelStar_0(:,obj.indexIteration) = full(input.controller.csdFn.mBodyAngVelStar(input.stateKinMBodyNoise.mBodyPosQuat_0,input.time));
            obj.motorsAngVelNoise(:,obj.indexIteration) = input.motorsAngVelNoise;
            
        end
        
        function data = getDataSI(obj,input)
            arguments
                obj
                input.model
            end
            umc = input.model.unitMeas.converter;
            data = obj.getDataSI@mystica.log.Logger('model',input.model);
            data.mBodyTwist_0(input.model.selector.indexes_mBodyLinVel_from_mBodyTwist,:) = data.mBodyTwist_0(input.model.selector.indexes_mBodyLinVel_from_mBodyTwist,:)./(umc.length); %[m/s]
        end
        
    end
end
