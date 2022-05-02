classdef LoggerKinAbs < mystica.log.Logger
    %LOGGERKINABS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mBodyTwist_0
        jointsAngVel_PJ
        mBodyAngVelStar_0
    end
    
    methods
        function obj = LoggerKinAbs(input)
            %LOGGERKINABS Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                input.model
                input.numberIterations
            end
            obj@mystica.log.Logger('model',input.model,'numberIterations',input.numberIterations)
            obj.mBodyAngVelStar_0 = zeros(input.model.constants.mBodyAngVel ,obj.numberIterations);
            obj.mBodyTwist_0      = zeros(input.model.constants.mBodyTwist  ,obj.numberIterations);
            obj.jointsAngVel_PJ   = zeros(input.model.constants.jointsAngVel,obj.numberIterations);
        end
        
        function store(obj,input)
            arguments
                obj
                input.time
                input.model
                input.indexIteration
                input.controller
                input.stgsDesiredShape
                input.mBodyTwist_0
                input.stateKinMBody
            end
            
            obj.storeStateKinMBody('model',input.model,'controller',input.controller,'indexIteration',input.indexIteration,...
                'stateKinMBody',input.stateKinMBody,'stgsDesiredShape',input.stgsDesiredShape,'time',input.time);
            
            obj.mBodyAngVelStar_0(:,obj.indexIteration) = full(input.controller.csdFn.mBodyAngVelStar(input.stateKinMBody.mBodyPosQuat_0,input.time));
            
            obj.mBodyTwist_0(:,obj.indexIteration)    = input.mBodyTwist_0;
            obj.jointsAngVel_PJ(:,obj.indexIteration) = input.stateKinMBody.referenceConversion.from_mBodyTwist0_2_jointsAngVelPJ * input.mBodyTwist_0;
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
