classdef CellJoint
    properties
        name
        indexChild
        indexParent
        connectionPoint = struct('parent',[],'child',[],'tform_pj0_cj0',[])
        jointType
        axesRotation
        axesActuated
        limitRoM
        limitJointVel
        limitJointTorque
        coeffViscousFriction
        coeffCoulombFriction
        coeffMotorTorque
        inertiaTensMotorRot_j_g
        transmissionGearRatio
        transmissionEfficiency
    end
    methods
        function obj = CellJoint()
        end
        function obj = fixAfterLinkRemoval(obj,indexLink)
            for fieldName = {'indexChild','indexParent'}
                if obj.(fieldName{:})>indexLink
                    obj.(fieldName{:}) = obj.(fieldName{:})-1;
                end
            end
        end
    end
end

