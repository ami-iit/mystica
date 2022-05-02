classdef Constants < matlab.mixin.Copyable
    
    properties (SetAccess=protected,GetAccess=public)
        linkPos    = 3;
        linkQuat   = 4;
        linkEul    = 3;
        linkAngVel = 3;
        linkTwist
        linkPosQuat
        mBodyTwist
        mBodyPosQuat
        mBodyLinVel
        mBodyAngVel
        mBodyPosVel
        jointsAngVel
        jointsAngVelBase
        motorsAngVel
        passiveAngVel
        constrainedAngVel
        nConstraints
        gravity
    end
    
    methods
        function obj = Constants(input)
            arguments
                input.nLink    = []
                input.nJoint   = []
                input.unitMeas struct
            end
            obj.gravity          = [0;0;-9.81]*(input.unitMeas.converter.length); %[m/s^2]*(input.unitMeas.converter.length)
            obj.linkTwist        = obj.linkPos + obj.linkEul;
            obj.linkPosQuat      = obj.linkPos + obj.linkQuat;
            if  isempty(input.nLink)==0
                obj = obj.getLinkConstants(input.nLink);
            end
            if  isempty(input.nJoint)==0
                obj = obj.getJointConstants(input.nJoint);
            end
        end
        function obj = getLinkConstants(obj,nLink)
            obj.mBodyTwist        = obj.linkTwist * nLink;
            obj.mBodyPosQuat      = obj.linkPosQuat * nLink;
            obj.mBodyLinVel       = obj.linkPos * nLink;
            obj.mBodyAngVel       = obj.linkEul * nLink;
            obj.mBodyPosVel       = obj.mBodyPosQuat + obj.mBodyTwist;
        end
        function obj = getJointConstants(obj,nJoint)
            obj.jointsAngVel     = obj.linkEul * nJoint;
            obj.jointsAngVelBase = obj.jointsAngVel + obj.linkTwist;
        end
        function obj = setNumberMotors(obj,numberMotors,numberPassiveJoints)
            obj.motorsAngVel      = numberMotors;
            obj.passiveAngVel     = numberPassiveJoints;
            obj.constrainedAngVel = obj.jointsAngVel - obj.motorsAngVel - obj.passiveAngVel;
        end
        function obj = setNumberConstraints(obj,nConstraints)
            obj.nConstraints = nConstraints;
        end
    end
end
