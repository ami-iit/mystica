classdef InertiaTensor

    methods
        function obj = InertiaTensor
        end
    end
    
    methods (Static)
        function I = solidCylinder(input)
            arguments
                input.diameter
                input.mass
                input.height
            end
            % cylinder is the one represented in https://upload.wikimedia.org/wikipedia/commons/thumb/d/d7/Moment_of_inertia_solid_cylinder.svg/300px-Moment_of_inertia_solid_cylinder.svg.png
            
            r = input.diameter/2;
            m = input.mass;
            h = input.height;
            
            Ixx = 1/12*m*(3*r^2+h^2);
            Iyy = Ixx;
            Izz = 1/2*m*r^2;
            
            I = diag([Ixx Iyy Izz]);
            
        end
        
        function I = tubeCylinder(input)
            arguments
                input.internalDiameter
                input.externalDiameter
                input.mass
                input.height
            end
            % cylinder is the one represented in https://upload.wikimedia.org/wikipedia/commons/c/cc/Moment_of_inertia_thick_cylinder_h.svg
            
            r1 = input.internalDiameter/2;
            r2 = input.externalDiameter/2;
            m = input.mass;
            h = input.height;
            
            Ixx = 1/12*m*(3*(r1^2+r2^2)+h^2);
            Iyy = Ixx;
            Izz = 1/2*m*(r1^2+r2^2);
            
            I = diag([Ixx Iyy Izz]);
            
        end
    end
end

