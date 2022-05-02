classdef Integrator < handle
    
    properties (SetAccess=protected,GetAccess=public)
        ti
        tf
        xi
        xf
        dxdt
    end
    properties (SetAccess=immutable,GetAccess=public)
        solverOpts
        dxdtOpts
        dxdtParam
        statusTracker
    end
    properties (SetAccess=private,GetAccess=private)
        ratioTimePrintMax       = 0
        ratioTimeTrackerFileMax = 0
        nameTTF = []
    end
    
    methods
        function obj = Integrator(input)
            arguments
                input.stgsIntegrator = struct();
                input.xi
                input.dxdt
                input.ti
                input.tf
            end
            fieldsStgsIntegrator = {'solverOpts','dxdtOpts','dxdtParam','statusTracker'};
            for i = 1 : length(fieldsStgsIntegrator)
                obj.(fieldsStgsIntegrator{i}) = obj.getfield(input.stgsIntegrator,fieldsStgsIntegrator{i});
            end
            obj.configureIntegrator(input);
            obj.xf = obj.xi;
        end
        
        function xf = integrate(obj,input)
            arguments
                obj
                input.xi   = obj.xf;
                input.dxdt = obj.dxdt;
                input.ti   = obj.ti;
                input.tf   = obj.tf;
            end
            obj.configureIntegrator(input)
            switch class(obj.dxdt)
                case 'casadi.Function'
                    odeCsd      = struct;
                    odeCsd.x    = casadi.MX.sym('x',size(obj.xi));
                    odeCsd.t    = casadi.MX.sym('t');
                    odeCsd.ode  = obj.dxdt(odeCsd.x);
                    optsCsd     = struct('t0',obj.ti,'tf',obj.tf);
                    I = casadi.integrator('I',obj.solverOpts.name,odeCsd,optsCsd);
                    r           = I('x0',obj.xi);
                    obj.xf      = full(r.xf);
                case 'function_handle'
                    opts = odeset;
                    list = {'AbsTol','RelTol'};
                    for i = 1 : length(list)
                        opts.(list{i}) = obj.getfield(obj.solverOpts,list{i});
                    end
                    ode    = str2func(obj.solverOpts.name);
                    [~,x]  = ode(obj.dxdt,[obj.ti obj.tf],obj.xi,opts);
                    obj.xf = transpose(x(end,:));
                otherwise
                    error('class(dxdt) not valid')
            end
            xf = obj.xf;
            obj.printWorkspaceStatus()
            obj.createTimeTrackerFile()
        end
        
        
    end
    
    methods (Access = protected)
        function configureIntegrator(obj,input)
            obj.xi   = input.xi;
            obj.dxdt = input.dxdt;
            obj.ti   = input.ti;
            obj.tf   = input.tf;
        end
        
        function printWorkspaceStatus(obj)
            if isempty(obj.statusTracker) == 0
                if obj.ratioTimePrintMax < floor(obj.tf*obj.statusTracker.workspacePrint.frameRate) && obj.statusTracker.workspacePrint.run
                    obj.ratioTimePrintMax = floor(obj.tf*obj.statusTracker.workspacePrint.frameRate);
                    fprintf('Integration Time: %.1f/%.0f\n',obj.tf,obj.statusTracker.limitMaximumTime);
                end
            end
        end
        
        function createTimeTrackerFile(obj)
            if isempty(obj.statusTracker) == 0
                if obj.ratioTimeTrackerFileMax < floor(obj.tf*obj.statusTracker.timeTrackerFile.frameRate) && obj.statusTracker.timeTrackerFile.run
                    obj.ratioTimeTrackerFileMax = floor(obj.tf*obj.statusTracker.timeTrackerFile.frameRate);
                    obj.nameTTF = mystica.utils.createTimeTrackerFile(obj.nameTTF,obj.statusTracker.timeTrackerFile.baseName,obj.tf,obj.statusTracker.limitMaximumTime);
                end
            end
        end
    end
    
    methods (Static,Access = protected)
        function val = getfield(s,name_field)
            if isfield(s,name_field)
                val = s.(name_field);
            else
                val = [];
            end
        end
    end
    
end
