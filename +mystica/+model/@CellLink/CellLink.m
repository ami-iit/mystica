classdef CellLink
    properties
        name
        mass = 0
        inertiaTens_g_g = zeros(3);
        tform_0_b
        tform_b_j
        tform_b_g = eye(4);
        baseLink
        fixed
        linkDimension
        stlFaceColor
        stlEdgeColor
        stlScale
        stlName
    end
    methods
        function obj = CellLink()
        end
        function obj = addRigidMass(obj,input)
            arguments
                obj
                input.massNewBody            (1,1) double = 0          %[kg]     % mass of the new rigid body
                input.inertiaTensNewBody_b_g (3,3) double = zeros(3)   %[kg*m^2] % inertia of the new body at its CENTER OF GRAVITY with respect to frame B
                input.pos_b_g                (3,1) double = zeros(3,1) %[m]      % CoM position wrt frame B
                input.rotm_b_G               (3,3) double = eye(3)     %SO3      % rotation from frame G (total body) wrt to frame B
            end
            
            % Body 1
            mB1       = obj.mass;
            pos_b_g1  = mystica.rbm.getPosGivenTform(obj.tform_b_g);
            rotm_b_g1 = mystica.rbm.getRotmGivenTform(obj.tform_b_g);
            iB1_b_g1  = rotm_b_g1*obj.inertiaTens_g_g*transpose(rotm_b_g1);
            % Body 2
            mB2       = input.massNewBody;
            pos_b_g2  = input.pos_b_g;
            iB2_b_g2  = input.inertiaTensNewBody_b_g;
            % Rotation matrix from frame G (bodies B1+B2) to frame B
            rotm_b_g = input.rotm_b_G;
            
            % total mass
            M = mB1 + mB2;
            
            % center of mass
            pos_b_g   = (pos_b_g1*mB1+pos_b_g2*mB2)/M;
            
            % total inertia
            rotm_g_b = transpose(rotm_b_g);
            pos_g_g1 = rotm_g_b*(pos_b_g1-pos_b_g);
            pos_g_g2 = rotm_g_b*(pos_b_g2-pos_b_g);
            iB1_g_g  = rotm_g_b*iB1_b_g1*rotm_b_g - mB1 * mystica.utils.skew(pos_g_g1)^2;
            iB2_g_g  = rotm_g_b*iB2_b_g2*rotm_b_g - mB2 * mystica.utils.skew(pos_g_g2)^2;
            iB_g_g   = iB1_g_g + iB2_g_g;
            
            obj.mass            = M;
            obj.tform_b_g       = mystica.rbm.getTformGivenPosRotm(pos_b_g,rotm_b_g);
            obj.inertiaTens_g_g = iB_g_g;
            
        end
    end
end
