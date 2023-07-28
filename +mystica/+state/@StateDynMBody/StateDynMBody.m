classdef StateDynMBody < mystica.state.StateKinMBody
    %STATEDYNMBODY Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess=protected,GetAccess=public)
        mBodyPosVel_0
    end
    properties (SetAccess=protected,GetAccess=protected)
        opti = []
        optiVar
    end

    methods
        function obj = StateDynMBody(input)
            arguments
                input.model
                input.mBodyPosQuat_0
                input.mBodyTwist_0
                input.stgsStateDynMBody
                input.stgsIntegrator
                input.stgsModel
            end

            obj@mystica.state.StateKinMBody('model',input.model,...
                'mBodyPosQuat_0',input.mBodyPosQuat_0,...
                'stgsStateKinMBody',input.stgsStateDynMBody)

            % Definition Casadi Variables
            obj.csdSy.motorsCurrent  = casadi.SX.sym('I',input.model.constants.motorsAngVel,1);
            obj.csdSy.mBodyPosVel_0  = casadi.SX.sym('chi',input.model.constants.mBodyPosVel,1);

            obj.getDynamicQuantities(input.model,input.stgsIntegrator,input.stgsModel);
            % generate MEX file containing (dyn) casadi funtions
            obj.generateMEX_dyn();
            % Evaluate initial state
            obj.setMBodyPosVel('mBodyPosVel_0',[input.mBodyPosQuat_0;input.mBodyTwist_0],'model',input.model)

        end
        [mBodyVelAcc_0,varargout] = get_mBodyVelAcc0_from_motorsCurrent(obj,input,stgs)

        function setMBodyPosVel(obj,input)
            arguments
                obj                 mystica.state.StateDynMBody
                input.mBodyPosVel_0 (:,1)
                input.model         mystica.model.Model
                input.method = 'full'
            end

            obj.mBodyPosVel_0  = input.mBodyPosVel_0;
            obj.setMBodyPosQuat('model',input.model,'mBodyPosQuat_0',obj.mBodyPosVel_0(input.model.selector.indexes_mBodyPosQuat_from_mBodyPosVel),'method',input.method)

        end

        function generateMEX(obj)
            obj.generateMEX@mystica.state.StateKinMBody();
            obj.generateMEX_dyn();
        end

    end
    methods (Access=protected)
        getDynamicQuantities(obj,model,stgsIntegrator,stgsModel)
    end
    methods (Access=private)
        function generateMEX_dyn(obj)
            nameMEX = 'mystica_stateDyn';
            opts = struct('main', true,'mex', true);
            initial_path = pwd;
            cd(fullfile(mystica.utils.getMysticaFullPath,'deps','csdMEX'));
            C = casadi.CodeGenerator([nameMEX,'.c'],opts);
            C.add(obj.csdFn.dJc);
            C.add(obj.csdFn.massMatrix);
            C.add(obj.csdFn.massMatrixMotorInertia);
            C.add(obj.csdFn.mBodyWrenchExt_0);
            C.add(obj.csdFn.mBodyWrenchGra_0);
            C.add(obj.csdFn.mBodyWrenchCor_0);
            C.add(obj.csdFn.mBodyWrenchFri_0);
            C.add(obj.csdFn.mBodyWrenchInp_0);
            C.add(obj.csdFn.from_motorsCurrent_2_mBodyWrenchInp0);
            %C.add(obj.csdFn.mBodyVelAcc_0); osqp cannot be code-generated
            C.generate();
            fileID = fopen([nameMEX,'.c']    ,'r'); new_code = fscanf(fileID,'%s'); fclose(fileID);
            if exist([nameMEX,'_old.c'],'file')
                fileID = fopen([nameMEX,'_old.c'],'r'); old_code = fscanf(fileID,'%s'); fclose(fileID);
            else
                old_code = '';
            end
            if ~strcmp(new_code,old_code) || isempty(dir([nameMEX,'.mex*']))
                fprintf('generating %s.mex\n',nameMEX)
                mex([nameMEX,'.c'],'-largeArrayDims')
            else
                fprintf('%s.mex already exists\n',nameMEX)
            end
            movefile([nameMEX,'.c'],[nameMEX,'_old.c'])
            cd(initial_path)
        end
    end
end
