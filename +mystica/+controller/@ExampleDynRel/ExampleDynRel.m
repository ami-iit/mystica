classdef ExampleDynRel < mystica.controller.Base

    properties (SetAccess=protected,GetAccess=public)
        motorsCurrent
    end

    methods
        function obj = ExampleDynRel(input)
            arguments
                input.model
                input.stateDynMBody
                input.stgsController
                input.stgsDesiredShape
                input.time
                input.controller_dt
            end
            obj@mystica.controller.Base(...
                'model',input.model,...
                'state',input.stateDynMBody,...
                'stgsController',input.stgsController,...
                'stgsDesiredShape',input.stgsDesiredShape,...
                'time',input.time,...
                'controller_dt',input.controller_dt);
            obj.motorsCurrent = ones(input.model.constants.motorsAngVel,1);
        end
        function motorsCurrent = solve(obj,input)
            arguments
                obj
                input.time
                input.stateDynMBody
                input.model
            end
            motorsCurrent     = rand(input.model.constants.motorsAngVel,1);
            obj.motorsCurrent = motorsCurrent;
        end
    end

end
