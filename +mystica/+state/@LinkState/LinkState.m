classdef LinkState < handle
    %LINK Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = private,GetAccess = public)
        csdFn
        tform_0_b
    end

    methods
        function obj = LinkState(input)
            %LINK Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                input.tform_0_b          = eye(4);
                input.csdMBodyPosQuat    = casadi.SX.sym('x',7,1)
                input.indexesLinkPosQuat = 1:7;
            end
            obj.tform_0_b = input.tform_0_b;
            obj.csdFn.tform_0_b = casadi.Function('tform_0_b',{input.csdMBodyPosQuat},{mystica.rbm.getTformGivenPosQuat(input.csdMBodyPosQuat(input.indexesLinkPosQuat))});
        end

        function setLinkStateGivenLinkPosQuat(obj,linkPosQuat)
            obj.tform_0_b = mystica.rbm.getTformGivenPosQuat(linkPosQuat);
        end

        function setLinkStateGivenLinkTform(obj,linkTform)
            obj.tform_0_b = linkTform;
        end
        
        function clearCasadi(obj)
            obj.csdFn = [];
        end

    end

end
