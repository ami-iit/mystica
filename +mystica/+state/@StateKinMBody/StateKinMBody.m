classdef StateKinMBody < matlab.mixin.Copyable
    %STATEKINMBODY Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess=protected,GetAccess=public)
        mBodyPosQuat_0
        mBodyPosQuat_0_initial
        Jc
        nullEvaluator
        nullJc_mBodyTwist_0
        nullJc_jointsAngVel_PJ
        linIndRowJc
        referenceConversion
        csdFn
        stgs
    end
    properties
        csdSy
    end

    methods
        function obj = StateKinMBody(input)
            arguments
                input.model          mystica.model.Model
                input.mBodyPosQuat_0 (:,1)
                input.stgsStateKinMBody
            end

            obj.stgs = input.stgsStateKinMBody;

            obj.csdSy.mBodyPosQuat_0 = casadi.SX.sym('x',input.model.constants.mBodyPosQuat,1);

            % links initialization
            obj.getJacobianConstraints(input.model)
            obj.getKinematicQuantities(input.model)
            obj.nullEvaluator = mystica.utils.NullSpace(...
                'decompositionMethod'   ,obj.stgs.nullSpace.decompositionMethod,...
                'rankRevealingMethod'   ,obj.stgs.nullSpace.rankRevealingMethod,...
                'toleranceRankRevealing',obj.stgs.nullSpace.toleranceRankRevealing);
            % generate MEX file containing casadi funtions
            obj.generateMEX();
            % Evaluate initial state
            obj.setMBodyPosQuat('mBodyPosQuat_0',input.mBodyPosQuat_0,'model',input.model)
            obj.mBodyPosQuat_0_initial = obj.mBodyPosQuat_0;
            input.model.constants.setNumberConstraints(size(obj.Jc,1)); % Note: @mystica.model.Model and @Constants are two handle classes! We are modifying model.constants in the main file
        end

        function clearProperties(obj)
            obj.csdFn = [];
            obj.csdSy = [];
        end

        function intJcV = getIntJcV(obj)
            intJcV = full(mystica_stateKin('intJcV',obj.mBodyPosQuat_0,obj.mBodyPosQuat_0_initial)); % obj.csdFn.intJcV(obj.mBodyPosQuat_0,obj.mBodyPosQuat_0_initial)
        end

        function generateMEX(obj)
            nameMEX = 'mystica_stateKin';
            opts = struct('main', true,'mex', true);
            initial_path = pwd;
            cd(fullfile(mystica.utils.getMysticaFullPath,'deps','csdMEX'));
            C = casadi.CodeGenerator([nameMEX,'.c'],opts);
            C.add(obj.csdFn.Jc);
            C.add(obj.csdFn.intJcV);
            C.add(obj.csdFn.rC_from_jointsAngVelPJ_2_jointsAngVel0);
            C.add(obj.csdFn.rC_from_mBodyTwist0_2_jointsAngVelPJ);
            C.add(obj.csdFn.get_mBodyVelQuat0_from_mBodyTwist0)
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

        function tform_b = get_link_tform_b(obj,input)
            arguments
                obj
                input.iLink (1,1) double
                input.model mystica.model.Model
                input.mBodyPosQuat_0 = obj.mBodyPosQuat_0
            end
            linkPosQuat_0 = input.mBodyPosQuat_0(input.model.linksAttributes{input.iLink}.selector.indexes_linkPosQuat_from_mBodyPosQuat);
            tform_b = mystica.rbm.getTformGivenPosQuat(linkPosQuat_0);
        end

        mBodyVelQuat    = get_mBodyVelQuat0_from_mBodyTwist0(obj,input);
        mBodyVelQuat    = get_mBodyVelQuat0_from_motorsAngVel(obj,input);
        mBodyTwist      = get_mBodyTwist0_from_motorsAngVel(obj,input)
        jointsAngVel_PJ = get_jointsAngVelPJ_from_motorsAngVel(obj,input);
        Zact = getZact(obj,input);
        setMBodyPosQuat(obj,input)
    end

    methods (Access=protected)
        getKinematicQuantities(obj,model)
        getJacobianConstraints(obj,model)
    end

end
