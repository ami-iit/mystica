classdef ExampleKinAbs < mystica.controller.Base

    properties (SetAccess=protected,GetAccess=public)
        mBodyTwist_0
    end

    methods
        function obj = ExampleKinAbs(input)
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
            obj.mBodyTwist_0 = zeros(input.model.constants.mBodyTwist,1);
        end

        function mBodyTwist_0 = solve(obj,input)
            arguments
                obj
                input.time
                input.stateKinMBody
                input.model
            end
            N = input.stateKinMBody.nullJc_mBodyTwist_0;
            mBodyTwist_0 = N*rand(size(N,2),1);
            obj.mBodyTwist_0 = mBodyTwist_0;
        end

    end
end
