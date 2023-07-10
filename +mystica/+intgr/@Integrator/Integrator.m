classdef Integrator < handle
    
    properties (SetAccess=protected,GetAccess=public)
        ti
        tf
        xi
        xf
        dxdt
    end
    properties (SetAccess=protected,GetAccess=protected)
        x
        t
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
            obj.x = [];
            obj.t = [];
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
                    obj.t = [obj.ti obj.tf];
                    obj.x = [obj.xi obj.xf];
                case 'function_handle'
                    opts = odeset;
                    list = {'AbsTol','RelTol','MaxStep'};
                    for i = 1 : length(list)
                        opts.(list{i}) = obj.getfield(obj.solverOpts,list{i});
                    end
                    ode    = str2func(obj.solverOpts.name);
                    [obj.t,obj.x]  = ode(obj.dxdt,[obj.ti obj.tf],obj.xi,opts);
                    obj.xf = transpose(obj.x(end,:));
                otherwise
                    error('class(dxdt) not valid')
            end
            xf = obj.xf;
            obj.printWorkspaceStatus()
            obj.createTimeTrackerFile()
        end
        
        function dxdt = get_dxdt(obj)
            arguments
                obj
            end
            if isempty(obj.x)
                dxdt = 0;
            elseif size(obj.x)==2
                dxdt = (obj.xf-obj.xi)/(obj.tf-obj.ti);
            else
                t = linspace(obj.t(1),obj.t(end),1e2);
                X = interp1(obj.t,obj.x,t);
                dt = t(2)-t(1);
                dxdt = zeros(size(X,1)-1,size(X,2));
                for i = 1 : size(dxdt,2)
                    dxdt(:,i) = smooth(diff(X(:,i))/dt,'sgolay',4);
                end
            end
            dxdt = dxdt(end,:)';
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
                    fprintf('Integration Time: %.2f/%.0f\n',obj.tf,obj.statusTracker.limitMaximumTime);
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
