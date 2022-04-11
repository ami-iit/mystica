classdef GearMotor
    
    properties
        motor
        transmission
        assembly
    end
    
    methods
        function obj = GearMotor()
        end
        
        function obj = setProperties(obj,in)
            obj.motor        = in.motor;
            obj.transmission = in.transmission;
            obj.assembly     = in.assembly;
        end
        
        function obj = computeProperties(obj,input)
            arguments
                obj
                input.ratioMassRotorStator  % k1 = mMR/mS; mM=mS+mMR
                input.ratioLengthRotorMotor % k2 = hMR/mM;
            end
            
            % see https://github.com/ami-iit/element_morphing-cover-design/issues/200#issuecomment-1033573930
            
            k1 = input.ratioMassRotorStator;
            k2 = input.ratioLengthRotorMotor;
            
            motor.stator.mass   = obj.motor.mass/(1+k1);
            motor.rotor.mass    = obj.motor.mass/(1+k1)*k1;
            
            motor.rotor.diameter = sqrt(8*obj.motor.rotor.inertia_j_g/motor.rotor.mass);
            motor.rotor.length = obj.motor.length*k2;
            
            motor.stator.inertia_j_g = mystica.rbm.InertiaTensor.tubeCylinder( 'mass',motor.stator.mass,'externalDiameter',obj.motor.diameter,'internalDiameter',motor.rotor.diameter,'height',obj.motor.length);
            motor.rotor.inertia_j_g  = mystica.rbm.InertiaTensor.solidCylinder('mass',motor.rotor.mass ,'diameter',motor.rotor.diameter,'height',motor.rotor.length);
            transmission.inertia_j_g = mystica.rbm.InertiaTensor.solidCylinder('mass',obj.transmission.mass,'diameter',obj.transmission.diameter ,'height',obj.transmission.length);
            
            % Assign Mass Property
            obj.assembly.rotor.inertia_j_g = diag([...
                motor.rotor.inertia_j_g(1,1)+transmission.inertia_j_g(1,1);...
                motor.rotor.inertia_j_g(2,2)+transmission.inertia_j_g(2,2);...
                motor.rotor.inertia_j_g(3,3)+transmission.inertia_j_g(3,3)*0]);
            obj.assembly.rotor.mass         = motor.rotor.mass+obj.transmission.mass;
            obj.assembly.stator.mass        = motor.stator.mass;
            obj.assembly.stator.inertia_j_g = motor.stator.inertia_j_g;
            
            % Assign Joint Limit
            obj.assembly.jointLimits.velocity = obj.minValid(obj.motor.limits.velocity,obj.transmission.limits.velocity)/obj.transmission.gear_ratio;
            obj.assembly.jointLimits.torque   = obj.minValid(obj.motor.limits.torque  *obj.transmission.gear_ratio,obj.transmission.limits.torque);
            
            % Remove
            obj.motor               = rmfield(obj.motor,'mass');
            obj.motor.rotor         = rmfield(obj.motor.rotor,'inertia_j_g');
            obj.motor.limits        = rmfield(obj.motor.limits,'torque');
            obj.motor.limits        = rmfield(obj.motor.limits,'velocity');
            obj.transmission        = rmfield(obj.transmission,'mass');
            obj.transmission.limits = rmfield(obj.transmission.limits,'torque');
            obj.transmission.limits = rmfield(obj.transmission.limits,'velocity');
            
        end
        
    end
    methods (Static,Access=protected)
        function m = minValid(a,b)
            if isempty(a)
                a = inf;
            end
            if isempty(b)
                b = inf;
            end
            m = min(a,b);
        end
        
    end
    
end
