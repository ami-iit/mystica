classdef LinkAttributes
    %LINK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name
        index
        joints
        mass
        inertiaTens_b_b
        tform_b_j
        tform_b_g
        fixed
        linkDimension
        visual
        selector
    end
    properties (Access=protected)
        tform_0_b
    end
    
    methods
        function obj = LinkAttributes(input)
            %LINK Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                input.name
                input.index
                input.joints
                input.mass
                input.inertiaTens_g_g = []
                input.inertiaTens_b_b = []
                input.tform_0_b
                input.tform_b_j
                input.tform_b_g
                input.fixed
                input.linkDimension
                input.stlName
                input.stlScale
                input.stlFaceColor
                input.stlEdgeColor
                input.constants
            end
            obj.index            = input.index;
            obj.joints           = input.joints;
            obj.mass             = input.mass;
            obj.tform_0_b        = input.tform_0_b;
            obj.tform_b_j        = input.tform_b_j;
            obj.tform_b_g        = input.tform_b_g;
            obj.fixed            = input.fixed;
            obj.linkDimension    = input.linkDimension;
            obj.selector         = createSelector(obj,input.constants);
            obj.visual.name      = input.stlName;
            obj.visual.scale     = input.stlScale;
            obj.visual.faceColor = input.stlFaceColor;
            obj.visual.edgeColor = input.stlEdgeColor;
            if isempty(input.inertiaTens_b_b)
                rotm_b_g = mystica.rbm.getRotmGivenTform(obj.tform_b_g);
                pos_b_g  = mystica.rbm.getPosGivenTform(obj.tform_b_g);
                obj.inertiaTens_b_b = rotm_b_g * input.inertiaTens_g_g * transpose(rotm_b_g) + obj.mass*mystica.utils.skew(pos_b_g)'*mystica.utils.skew(pos_b_g);
            else
                obj.inertiaTens_b_b = input.inertiaTens_b_b;
            end
            if isempty(input.name)
                obj.name = ['link',num2str(obj.index)];
            else
                obj.name = input.name;
            end
        end
    end
    methods (Access=protected)
        selector = createSelector(obj,constants)
    end
end
