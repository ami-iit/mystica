function [umc,unitMeas] = setUM(input)
    arguments
        input.length char = 'm'
        input.mass   char = 'kg'
    end

    unitMeas.length = input.length;
    unitMeas.mass   = input.mass;
    
    switch unitMeas.length
        case 'm'
            umc.length = 1e0; %[m/m]
        case 'dm'
            umc.length = 1e1; %[dm/m]
        case 'cm'
            umc.length = 1e2; %[cm/m]
        case 'mm'
            umc.length = 1e3; %[mm/m]
    end
    
    switch unitMeas.mass
        case 'kg'
            umc.mass = 1e0; %[kg/kg]
        case 'hg'
            umc.mass = 1e1; %[hg/kg]
        case 'dag'
            umc.mass = 1e2; %[dag/kg]
        case 'g'
            umc.mass = 1e3; %[g/kg]
    end
    
    umc.force  = umc.mass*umc.length; 
    umc.torque = umc.mass*umc.length.^2;

    unitMeas.converter = umc;

end
