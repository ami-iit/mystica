classdef JointActuated < mystica.model.Joint
    %JOINTACTUATED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        inertiaTensMotorRot_j_g
        transmission
    end
    
    methods
        function obj = JointActuated(input)
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
                input.limitRoM                (1,1)
                input.limitJointVel           (3,1)
                input.limitJointTorque        (3,1)
                input.coeffViscousFriction    (3,1)
                input.coeffCoulombFriction    (3,1)
                input.coeffMotorTorque        (3,1)
                input.inertiaTensMotorRot_j_g (3,1)
                input.transmissionGearRatio   (3,1) % motor speed / joint speed
                input.transmissionEfficiency  (3,1)
            end
            
            obj@mystica.model.Joint(...
                'name'                 ,input.name,...
                'linkParent'           ,input.linkParent,...
                'linkChild'            ,input.linkChild,...
                'connectionPointParent',input.connectionPointParent,...
                'connectionPointChild' ,input.connectionPointChild,...
                'index'                ,input.index,...
                'type'                 ,input.type,...
                'axesRotation'         ,input.axesRotation,...
                'axesActuated'         ,input.axesActuated,...
                'tform_pj0_cj0'        ,input.tform_pj0_cj0,...
                'constants'            ,input.constants,...
                'limitRoM'             ,input.limitRoM,...
                'limitJointVel'        ,input.limitJointVel,...
                'limitJointTorque'     ,input.limitJointTorque,...
                'coeffViscousFriction' ,input.coeffViscousFriction,...
                'coeffCoulombFriction' ,input.coeffCoulombFriction)
            
            % we assume axis of rotation decoupled -> diagonal matrices
            obj.coefficients.motorTorque = sparse(diag(input.coeffMotorTorque       .*obj.axesActuated));
            obj.inertiaTensMotorRot_j_g  = sparse(diag(input.inertiaTensMotorRot_j_g.*obj.axesActuated));
            obj.transmission.gearRatio   = sparse(diag(input.transmissionGearRatio  .*obj.axesActuated));
            obj.transmission.efficiency  = sparse(diag(input.transmissionEfficiency .*obj.axesActuated));
            
            if any((obj.axesRotation-obj.axesActuated)<0)
                error('You are trying to actuate a fixed axis of rotation')
            end
            
        end
        
    end
end
