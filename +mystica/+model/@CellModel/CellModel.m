classdef CellModel < matlab.mixin.Copyable
    properties (SetAccess=protected,GetAccess=public)
        cellAdjacency
        cellLinks
        unitMeas
        name
    end
    properties (SetAccess=protected,GetAccess=protected)
        visual
        specs
    end
    properties (Dependent)
        nLink
        nJoint
    end

    methods
        function obj = CellModel(input)
            arguments
                input.nameModel = '';
                input.unitMeas  = [];
            end
            if isempty(input.unitMeas)
                [~,input.unitMeas] = mystica.model.setUM();
            end
            obj.unitMeas = input.unitMeas;
            obj.name     = input.nameModel;
        end

        function initialize(obj,input)
            arguments
                obj
                input.nLinks (1,1) double
            end
            obj.cellLinks{input.nLinks} = {};
            for i = 1 : input.nLinks
                obj.cellLinks{i} = mystica.model.CellLink();
                % implemented with a for cycle to avoid the creation of
                % references in case `class` CellLink will be converted to
                % an handle class
            end
            obj.cellAdjacency{input.nLinks,input.nLinks} = {};
        end

        %-----------------------------------------------------------------%
        % Link

        function nLink = get.nLink(obj)
            nLink = length(obj.cellLinks);
        end

        function assignLinkProperty(obj,input)
            arguments
                obj
                input.indexes double = []
                input.name char
                input.value
            end
            if isempty(input.indexes)
                input.indexes = 1:length(obj.cellLinks);
            end
            for i = input.indexes(:)'
                obj.cellLinks{i}.(input.name) = input.value;
            end
        end

        function assignLinkLogicalProperty(obj,input)
            arguments
                obj
                input.indexes double = []
                input.name char
                input.value
            end
            for i = 1:length(obj.cellLinks)
                if any(i==input.indexes)
                    obj.cellLinks{i}.(input.name) = ~~input.value;
                else
                    obj.cellLinks{i}.(input.name) =  ~input.value;
                end
            end
        end

        function addRigidMassToLink(obj,input)
            arguments
                obj
                input.indexLink
                input.massNewBody            = 0
                input.pos_b_g                = zeros(3,1)
                input.inertiaTensNewBody_b_g = zeros(3)
                input.rotm_b_g               = eye(3)
            end
            obj.cellLinks{input.indexLink} = obj.cellLinks{input.indexLink}.addRigidMass(...
                'massNewBody',input.massNewBody,...
                'pos_b_g',input.pos_b_g,...
                'inertiaTensNewBody_b_g',input.inertiaTensNewBody_b_g,...
                'rotm_b_g',input.rotm_b_g);
        end

        function removeLink(obj,input)
            arguments
                obj
                input.indexLink (1,1) double
            end
            obj.cellAdjacency(input.indexLink,:) = [];
            obj.cellAdjacency(:,input.indexLink) = [];
            obj.cellLinks(input.indexLink) = [];
            for j = find(~cellfun('isempty',obj.cellAdjacency))'
                obj.cellAdjacency{j} = obj.cellAdjacency{j}.fixAfterLinkRemoval(input.indexLink);
            end
        end

        %-----------------------------------------------------------------%
        % Joint

        function nJoint = get.nJoint(obj)
            nJoint = sum(~cellfun('isempty',obj.cellAdjacency),'all')/2;
        end

        function assignJointPropertyGivenFamily(obj,input)
            arguments
                obj
                input.link1 (1,1) double
                input.link2 (1,1) double
                input.name char
                input.value
            end
            if isa(obj.cellAdjacency{input.link1,input.link2},'mystica.model.CellJoint')==0
                obj.cellAdjacency{input.link1,input.link2} = mystica.model.CellJoint();
            end
            if isa(obj.cellAdjacency{input.link2,input.link1},'mystica.model.CellJoint')==0
                obj.cellAdjacency{input.link2,input.link1} = mystica.model.CellJoint();
            end
            obj.cellAdjacency{input.link1,input.link2}.(input.name) = input.value;
            obj.cellAdjacency{input.link2,input.link1}.(input.name) = input.value;
        end

        function assignJointProperty(obj,input)
            arguments
                obj
                input.name char
                input.value
                input.mask logical = 1
            end
            input.mask = input.mask | input.mask';
            for j = find(~cellfun('isempty',obj.cellAdjacency)&input.mask)'
                if isa(obj.cellAdjacency{j},'mystica.model.CellJoint')==0
                    obj.cellAdjacency{j} = mystica.model.CellJoint();
                end
                obj.cellAdjacency{j}.(input.name) = input.value;
            end
        end

        function removeJointGivenFamily(obj,input)
            arguments
                obj
                input.link1 (1,1) double
                input.link2 (1,1) double
            end
            obj.cellAdjacency{input.link1,input.link2} = {};
            obj.cellAdjacency{input.link2,input.link1} = {};
        end

        %-----------------------------------------------------------------%

        varargout = createMeshDataFE(obj,mesh,visual,femData,debug)
        createMeshTriangleNodePatternDiamond(obj,mesh)
        createMeshTriangleNodePatternRectangle(obj,mesh)

    end

    methods (Static)
        [angleH,angleV,h] = getYoshimuraBucklingDimensions(d,m)
    end

end
