classdef Logger < matlab.mixin.Copyable
    %LOGGER Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess=protected,GetAccess=public)
        mBodyPosQuat_0
        jointsAngEul_PJ
        jointsAngRotZ_PJ
        errorOrientationNormals
        errorPositionNormals
        nDoF
        time
        detZact
        nCondZact
        nCondJcRed
        sizeJcRed
        intJcV
    end
    properties (SetAccess=immutable,GetAccess=protected)
        numberIterations
    end
    properties (SetAccess=protected,GetAccess=protected)
        indexIteration
    end
    methods
        function obj = Logger(input)
            arguments
                input.model
                input.numberIterations
            end
            %LOGGER Construct an instance of this class
            %   Detailed explanation goes here
            obj.numberIterations  = input.numberIterations;
            obj.time              = zeros(1,obj.numberIterations);
            obj.nDoF              = zeros(1,obj.numberIterations);
            obj.detZact           = zeros(1,obj.numberIterations);
            obj.nCondZact         = zeros(1,obj.numberIterations);
            obj.nCondJcRed        = zeros(1,obj.numberIterations);
            obj.sizeJcRed         = zeros(2,obj.numberIterations);
            obj.mBodyPosQuat_0    = zeros(input.model.constants.mBodyPosQuat,obj.numberIterations);
            obj.jointsAngEul_PJ   = zeros(input.model.constants.jointsAngVel,obj.numberIterations);
            obj.jointsAngRotZ_PJ  = zeros(input.model.nJoint,obj.numberIterations);
            obj.errorOrientationNormals = zeros(input.model.nLink,obj.numberIterations);
            obj.errorPositionNormals    = zeros(input.model.nLink,obj.numberIterations);
            obj.intJcV = zeros(input.model.constants.nConstraints,obj.numberIterations);
        end

        function storeStateKinMBody(obj,input)
            arguments
                obj
                input.time
                input.model
                input.indexIteration
                input.stateKinMBody
                input.controller
                input.stgsDesiredShape
            end
            obj.indexIteration = input.indexIteration;
            obj.time(:,obj.indexIteration) = input.time;
            obj.nDoF(:,obj.indexIteration) = size(input.stateKinMBody.nullJc_mBodyTwist_0,2);
            obj.mBodyPosQuat_0(:,obj.indexIteration) = input.stateKinMBody.mBodyPosQuat_0;

            Zact = input.stateKinMBody.getZact('model',input.model);
            if size(Zact,1) == size(Zact,2)
                obj.detZact(:,obj.indexIteration) = det(Zact);
            else
                obj.detZact(:,obj.indexIteration) = 0;
            end

            obj.nCondZact( :,obj.indexIteration) = cond(Zact);
            obj.nCondJcRed(:,obj.indexIteration) = cond(full(input.stateKinMBody.Jc(input.stateKinMBody.linIndRowJc,:)));
            obj.sizeJcRed( :,obj.indexIteration) = size(input.stateKinMBody.Jc(input.stateKinMBody.linIndRowJc,:));

            obj.errorOrientationNormals(:,obj.indexIteration) = 180/pi*real(full(input.controller.csdFn.mBodyErrorOrientationNormal(input.stateKinMBody.mBodyPosQuat_0,input.time)));
            obj.errorPositionNormals(:,obj.indexIteration)    = full(input.controller.csdFn.mBodyErrorPositionNormal(input.stateKinMBody.mBodyPosQuat_0,input.time));

            obj.intJcV(:,obj.indexIteration) = input.stateKinMBody.getIntJcV;

            rotm_0_b{input.model.nLink} = [];
            for i = 1 : input.model.nLink
                rotm_0_b{i} = mystica.rbm.getRotmGivenTform(input.stateKinMBody.get_link_tform_b('iLink',i,'model',input.model));
            end

            for j = 1 : input.model.nJoint
                index = input.model.joints{j}.getJointConnectionDetails;
                rotm_0_p   = rotm_0_b{index.parent};
                rotm_0_c   = rotm_0_b{index.child};
                rotm_p_pj  = mystica.rbm.getRotmGivenTform(input.model.linksAttributes{index.parent}.tform_b_j{index.cPointParent});
                rotm_c_cj  = mystica.rbm.getRotmGivenTform(input.model.linksAttributes{index.child }.tform_b_j{index.cPointChild });
                rotm_pj_cj = rotm_p_pj'*rotm_0_p'*rotm_0_c*rotm_c_cj;
                rzyx       = mystica.rbm.getEulZYXGivenRotm(rotm_pj_cj);
                obj.jointsAngEul_PJ(input.model.joints{j}.selector.indexes_jAngVel_from_jointsAngVel,obj.indexIteration) = [rzyx(3);rzyx(2);rzyx(1)];
                obj.jointsAngRotZ_PJ(j,obj.indexIteration) = acos(mystica.utils.saturateSignal(rotm_pj_cj(1,1),1,-1));
            end

        end

        function data = getDataSI(obj,input)
            arguments
                obj
                input.model
            end
            umc = input.model.unitMeas.converter;
            data = obj.copy();
            data.mBodyPosQuat_0(input.model.selector.indexes_mBodyPos_from_mBodyPosQuat,:) = data.mBodyPosQuat_0(input.model.selector.indexes_mBodyPos_from_mBodyPosQuat,:)./umc.length; %[m]
            data.errorPositionNormals = data.errorPositionNormals./umc.length; %[m]
            data.intJcV(input.model.selector.indexes_constrainedLinVel_from_JcV,:) = data.intJcV(input.model.selector.indexes_constrainedLinVel_from_JcV,:)./umc.length; %[m/s];
        end

        dataOut = merge(obj,dataIn,stgs);

    end
end
