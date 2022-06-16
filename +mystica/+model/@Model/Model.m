classdef Model < matlab.mixin.Copyable
    %MODEL Summary of this class goes here
    %   Detailed explanation goes here

    properties
        adjacencyMatrix
        constants
    end
    properties (SetAccess=immutable,GetAccess=public)
        nJoint (1,1)
        nLink  (1,1)
        indexBase = 0
        indexesFixedLink
        unitMeas
        name
    end
    properties (SetAccess=protected,GetAccess=public)
        joints
        linksAttributes
        selector
    end
    properties (SetAccess=immutable,GetAccess=private)
        graph
    end
    properties (SetAccess=protected,GetAccess=private)
        matrixParentChild
    end

    methods
        function obj = Model(cellModel)
            cellAdjacency = cellModel.cellAdjacency;
            cellLinks     = cellModel.cellLinks;
            obj.name      = cellModel.name;
            obj.unitMeas  = cellModel.unitMeas;
            obj.getAdjacencyMatrix(cellAdjacency);
            obj.graph = graph(obj.adjacencyMatrix~=0);
            obj.nLink = size(obj.adjacencyMatrix,1);
            obj.nJoint = size(obj.graph.Edges,1);
            obj.constants = mystica.model.Constants('nLink',obj.nLink,'nJoint',obj.nJoint,'unitMeas',obj.unitMeas);
            obj.matrixParentChild = zeros(obj.nJoint,2);

            % Links
            obj.linksAttributes{obj.nLink} = {};
            for i = 1 : obj.nLink
                nJointArray = transpose(1:obj.nJoint);
                obj.linksAttributes{i} = mystica.model.LinkAttributes(...
                    'index'          ,i,...
                    'name'           ,cellLinks{i}.name,...
                    'joints'         ,nJointArray(any(obj.graph.Edges.EndNodes == i,2)),...
                    'mass'           ,cellLinks{i}.mass,...
                    'inertiaTens_g_g',cellLinks{i}.inertiaTens_g_g,...
                    'tform_0_b'      ,cellLinks{i}.tform_0_b,...
                    'tform_b_j'      ,cellLinks{i}.tform_b_j,...
                    'tform_b_g'      ,cellLinks{i}.tform_b_g,...
                    'fixed'          ,cellLinks{i}.fixed,...
                    'linkDimension'  ,cellLinks{i}.linkDimension,...
                    'stlFaceColor'   ,cellLinks{i}.stlFaceColor,...
                    'stlEdgeColor'   ,cellLinks{i}.stlEdgeColor,...
                    'stlScale'       ,cellLinks{i}.stlScale,...
                    'stlName'        ,cellLinks{i}.stlName,...
                    'constants'      ,obj.constants);

                obj.indexBase     = (cellLinks{i}.baseLink == 1) * i + (cellLinks{i}.baseLink == 0) * obj.indexBase;

                if cellLinks{i}.fixed
                    obj.indexesFixedLink = [obj.indexesFixedLink; i];
                end
            end

            % joints
            obj.joints{obj.nJoint} = {};
            for i = 1 : obj.nJoint
                indexChild  = cellAdjacency{obj.graph.Edges.EndNodes(i,1),obj.graph.Edges.EndNodes(i,2)}.indexChild;
                indexParent = cellAdjacency{obj.graph.Edges.EndNodes(i,1),obj.graph.Edges.EndNodes(i,2)}.indexParent;
                obj.matrixParentChild(i,:) = [indexParent indexChild];
                if any(cellAdjacency{indexParent,indexChild}.axesActuated)
                    obj.joints{i} = mystica.model.JointActuated(...
                        'name'                   ,cellAdjacency{indexParent,indexChild}.name,...
                        'linkChild'              ,indexChild,...
                        'linkParent'             ,indexParent,...
                        'connectionPointChild'   ,cellAdjacency{indexParent,indexChild}.connectionPoint.child,...
                        'connectionPointParent'  ,cellAdjacency{indexParent,indexChild}.connectionPoint.parent,...
                        'index'                  ,i,...
                        'type'                   ,cellAdjacency{indexParent,indexChild}.jointType,...
                        'axesRotation'           ,cellAdjacency{indexParent,indexChild}.axesRotation,...
                        'axesActuated'           ,cellAdjacency{indexParent,indexChild}.axesActuated,...
                        'tform_pj0_cj0'          ,cellAdjacency{indexParent,indexChild}.connectionPoint.tform_pj0_cj0,...
                        'constants'              ,obj.constants,...
                        'limitRoM'               ,cellAdjacency{indexParent,indexChild}.limitRoM,...
                        'limitJointVel'          ,cellAdjacency{indexParent,indexChild}.limitJointVel,...
                        'limitJointTorque'       ,cellAdjacency{indexParent,indexChild}.limitJointTorque,...
                        'coeffViscousFriction'   ,cellAdjacency{indexParent,indexChild}.coeffViscousFriction,...
                        'coeffCoulombFriction'   ,cellAdjacency{indexParent,indexChild}.coeffCoulombFriction,...
                        'coeffMotorTorque'       ,cellAdjacency{indexParent,indexChild}.coeffMotorTorque,...
                        'inertiaTensMotorRot_j_g',cellAdjacency{indexParent,indexChild}.inertiaTensMotorRot_j_g,...
                        'transmissionGearRatio'  ,cellAdjacency{indexParent,indexChild}.transmissionGearRatio,...
                        'transmissionEfficiency' ,cellAdjacency{indexParent,indexChild}.transmissionEfficiency);
                else
                    obj.joints{i} = mystica.model.Joint(...
                        'name'                  ,cellAdjacency{indexParent,indexChild}.name,...
                        'linkChild'             ,indexChild,...
                        'linkParent'            ,indexParent,...
                        'connectionPointChild'  ,cellAdjacency{indexParent,indexChild}.connectionPoint.child,...
                        'connectionPointParent' ,cellAdjacency{indexParent,indexChild}.connectionPoint.parent,...
                        'index'                 ,i,...
                        'type'                  ,cellAdjacency{indexParent,indexChild}.jointType,...
                        'axesRotation'          ,cellAdjacency{indexParent,indexChild}.axesRotation,...
                        'axesActuated'          ,cellAdjacency{indexParent,indexChild}.axesActuated,...
                        'tform_pj0_cj0'         ,cellAdjacency{indexParent,indexChild}.connectionPoint.tform_pj0_cj0,...
                        'constants'             ,obj.constants,...
                        'limitRoM'              ,cellAdjacency{indexParent,indexChild}.limitRoM,...
                        'limitJointVel'         ,cellAdjacency{indexParent,indexChild}.limitJointVel,...
                        'limitJointTorque'      ,cellAdjacency{indexParent,indexChild}.limitJointTorque,...
                        'coeffViscousFriction'  ,cellAdjacency{indexParent,indexChild}.coeffViscousFriction,...
                        'coeffCoulombFriction'  ,cellAdjacency{indexParent,indexChild}.coeffCoulombFriction);
                end
            end
            obj.configure();
        end

        function appendSelectorConstrainedDirections(obj,input)
            arguments
                obj
                input.indexes_ang = []
                input.indexes_lin = []
            end
            obj.selector.indexes_constrainedAngVel_from_JcV = unique([obj.selector.indexes_constrainedAngVel_from_JcV ; input.indexes_ang(:)]);
            obj.selector.indexes_constrainedLinVel_from_JcV = unique([obj.selector.indexes_constrainedLinVel_from_JcV ; input.indexes_lin(:)]);
        end

        function jointIndex = getJointIndex(obj,parentIndex,childIndex)
            positionBool = all(obj.matrixParentChild == [parentIndex childIndex],2);
            tempVector = 1 : length(positionBool);
            jointIndex = tempVector(positionBool);
        end

        function mBodyPosQuat_0 = getMBodyPosQuatRestConfiguration(obj)
            mBodyPosQuat_0 = zeros(obj.constants.mBodyPosQuat,1);
            for i = 1 : obj.nLink
                mBodyPosQuat_0(obj.linksAttributes{i}.selector.indexes_linkPosQuat_from_mBodyPosQuat) = mystica.rbm.getPosQuatGivenTform(obj.linksAttributes{i}.tform_0_b);
            end
        end

        function visualizeGraph(obj)
            figure
            plot(obj.graph,'EdgeLabel',1:obj.nJoint,'EdgeLabelColor',[0 0.5 0.5],'MarkerSize',5,'NodeFontSize',10)
        end

        function changeLinkProperty(obj,input)
            arguments
                obj
                input.indexes = 1:obj.nLink
                input.name char
                input.value
            end
            for i = input.indexes(:)'
                obj.linksAttributes{i}.(input.name) = input.value;
            end
        end

        function changeJointProperty(obj,input)
            arguments
                obj
                input.indexes = 1:obj.nJoint
                input.name char
                input.value
            end
            for i = input.indexes(:)'
                obj.joints{i}.(input.name) = input.value;
            end
        end

        function actuateJoint(obj,input)
            arguments
                obj
                input.jointIndex              (1,1)
                input.axesActuated            (3,1)
                input.coeffMotorTorque        (3,1) = ones(3,1)
                input.inertiaTensMotorRot_j_g (3,1) = zeros(3,1)
                input.transmissionGearRatio   (3,1) = ones(3,1)
                input.transmissionEfficiency  (3,1) = ones(3,1)
                input.byPassWarning                 = false;
            end
            i = input.jointIndex;
            if isa(obj.joints{i},'mystica.model.JointActuated') && ~input.byPassWarning
                warning('class(joint{%i}) = "mystica.model.JointActuated" \nJoint %i is already actuated. Data will be overriden',i,i)
            end
            obj.joints{i} = mystica.model.JointActuated(...
                'name'                   ,obj.joints{i}.name,...
                'linkChild'              ,obj.joints{i}.linkChild,...
                'linkParent'             ,obj.joints{i}.linkParent,...
                'connectionPointChild'   ,obj.joints{i}.connectionPointChild,...
                'connectionPointParent'  ,obj.joints{i}.connectionPointParent,...
                'index'                  ,obj.joints{i}.index,...
                'type'                   ,obj.joints{i}.type,...
                'axesRotation'           ,obj.joints{i}.axesRotation,...
                'axesActuated'           ,input.axesActuated,...
                'tform_pj0_cj0'          ,obj.joints{i}.tform_pj0_cj0,...
                'constants'              ,obj.constants,...
                'limitRoM'               ,obj.joints{i}.limitRoM,...
                'limitJointVel'          ,obj.joints{i}.limitJointVel,...
                'limitJointTorque'       ,obj.joints{i}.limitJointTorque,...
                'coeffViscousFriction'   ,diag(obj.joints{i}.coefficients.viscousFriction),...
                'coeffCoulombFriction'   ,diag(obj.joints{i}.coefficients.coulombFriction),...
                'coeffMotorTorque'       ,input.coeffMotorTorque,...
                'inertiaTensMotorRot_j_g',input.inertiaTensMotorRot_j_g,...
                'transmissionGearRatio'  ,input.transmissionGearRatio,...
                'transmissionEfficiency' ,input.transmissionEfficiency);
            obj.configure();
        end

    end

    methods (Access = private)
        getAdjacencyMatrix(obj,cellAdjacency)
        setMBodyRestConfiguration(obj)
        function configure(obj)
            nMotors  = 0;
            nPassive = 0;
            for i = 1 : obj.nJoint
                nMotors  = nMotors  + sum( obj.joints{i}.axesActuated);
                nPassive = nPassive + sum(~obj.joints{i}.axesActuated & obj.joints{i}.axesRotation);
            end
            obj.constants = obj.constants.setNumberMotors(nMotors,nPassive);
            obj.setMBodyRestConfiguration()
            obj.createSelector()
        end
    end

end
