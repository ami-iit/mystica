classdef Joint
    %JOINT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess=protected,GetAccess=public)
        name
        index
        linkParent
        linkChild
        type
        axesRotation
        axesActuated
        connectionPointParent
        connectionPointChild
        tform_pj0_cj0
        selector
    end
    properties (SetAccess=protected,GetAccess=public)
        visual
        coefficients
        limitRoM
        limitJointVel
        limitJointTorque
    end
    
    methods
        function obj = Joint(input)
            %JOINT Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                input.name
                input.linkParent (1,1)
                input.linkChild  (1,1)
                input.connectionPointParent (1,1)
                input.connectionPointChild  (1,1)
                input.index (1,1)
                input.type  char
                input.axesRotation  (3,1)
                input.axesActuated  (3,1)
                input.tform_pj0_cj0 (4,4)
                input.constants
                input.limitRoM               (1,1)
                input.limitJointVel          (3,1)
                input.limitJointTorque       (3,1)
                input.coeffViscousFriction   (3,1)
                input.coeffCoulombFriction   (3,1)
            end
            
            obj.linkParent       = input.linkParent;
            obj.linkChild        = input.linkChild;
            obj.index            = input.index;
            obj.type             = input.type;
            obj.axesRotation     = input.axesRotation;
            obj.axesActuated     = input.axesActuated;
            obj.limitRoM         = input.limitRoM;
            obj.limitJointVel    = input.limitJointVel.*obj.axesRotation;
            obj.limitJointTorque = input.limitJointTorque  .*obj.axesRotation;
            
            obj.coefficients.viscousFriction = sparse(diag(input.coeffViscousFriction  .*obj.axesRotation));
            obj.coefficients.coulombFriction = sparse(diag(input.coeffCoulombFriction  .*obj.axesRotation));
            
            obj.connectionPointParent = input.connectionPointParent;
            obj.connectionPointChild  = input.connectionPointChild;
            obj.tform_pj0_cj0         = input.tform_pj0_cj0;
            obj.selector              = createSelector(obj,input.constants);
            
            if isempty(input.name)
                obj.name = ['joint',num2str(obj.index)];
            else
                obj.name = input.name;
            end
            
            
        end
        
        function index = getJointConnectionDetails(obj)
            index.parent       = obj.linkParent;
            index.child        = obj.linkChild;
            index.cPointParent = obj.connectionPointParent;
            index.cPointChild  = obj.connectionPointChild;
        end
        
    end
    
    methods (Access=protected)
        selector = createSelector(obj,constants)
    end
end
