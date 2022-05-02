classdef Base < handle
    properties (SetAccess=protected,GetAccess=public)
        time
        stgsController
        stgsDesiredShape
        dt
        opti
        csdFn
    end
    properties (SetAccess=protected,GetAccess=protected)
        csdSy
    end
    methods (Access=public)
        function obj = Base(input)
            arguments
                input.model
                input.state
                input.stgsController
                input.stgsDesiredShape
                input.time
                input.controller_dt
            end
            obj.stgsDesiredShape = input.stgsDesiredShape;
            obj.stgsController   = input.stgsController;
            obj.time             = input.time;
            obj.dt               = input.controller_dt;
            obj.opti = casadi.Opti(obj.stgsController.casadi.optimizationType);
            p_opts = struct('expand',true,'error_on_fail',false);
            s_opts = struct();
            obj.opti.solver(obj.stgsController.casadi.solver,p_opts,s_opts);
            obj.csdSy.mBodyPosQuat_0       = obj.opti.parameter(input.model.constants.mBodyPosQuat,1);
            obj.csdSy.time                 = obj.opti.parameter(1);
            obj.getLinkFunctions(input.model,input.stgsDesiredShape.fun,input.stgsDesiredShape.invertNormals);
        end
        function clearProperties(obj)
            obj.opti  = [];
            obj.csdFn = [];
            obj.csdSy = [];
        end
    end
    methods (Access=protected)
        getLinkFunctions(obj,model,funDesiredShape,invertNormal)
    end
end
