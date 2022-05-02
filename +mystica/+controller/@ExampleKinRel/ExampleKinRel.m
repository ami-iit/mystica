classdef ExampleKinRel < mystica.controller.Base

    properties (SetAccess=protected,GetAccess=public)
        motorsAngVel
    end

    methods
        function obj = ExampleKinRel(input)
            arguments
                input.model
                input.stateKinMBody
                input.stgsController
                input.stgsDesiredShape
                input.time
                input.controller_dt
            end
            obj@mystica.controller.Base(...
                'model',input.model,...
                'state',input.stateKinMBody,...
                'stgsController',input.stgsController,...
                'stgsDesiredShape',input.stgsDesiredShape,...
                'time',input.time,...
                'controller_dt',input.controller_dt);
            obj.motorsAngVel = zeros(input.model.constants.motorsAngVel,1);
        end
        function motorsAngVel = solve(obj,input)
            arguments
                obj
                input.time
                input.stateKinMBody
                input.model
            end
            motorsAngVel     = rand(input.model.constants.motorsAngVel,1);
            obj.motorsAngVel = motorsAngVel;
        end
    end
end
