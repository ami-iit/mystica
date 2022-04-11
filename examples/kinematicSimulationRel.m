%% kinematic Simulation controlling motorsAngVel

clear
clc
close all
fclose('all');

% generate Model

switch '4barLinkage'
    case 'flat_squareLinks'
        model = mystica.model.getModelCoverSquareLinks('n',6,'m',3,'restConfiguration','cylinder','linkDimension',0.0425);
        % assign motors location
        booleanActuatedJointAngVel = zeros(model.constants.jointsAngVel,1); 
        booleanActuatedJointAngVel([3,6,9,15,16,21,24,27,30,33,36,45,48,51,53,60,66,69,75,78,81]) = 1;
        booleanActuatedJointAngVel = reshape(booleanActuatedJointAngVel,model.constants.linkEul,[]);
        for j = 1:model.nJoint
            model.actuateJoint('jointIndex',j,'axesActuated',booleanActuatedJointAngVel(:,j),'byPassWarning',1);
        end
        clear booleanActuatedJointAngVel
    case '4barLinkage'
        model = mystica.model.getModel4BarLinkage();
end

% define simulation settings and parameters

stgs = mystica.stgs.getDefaultSettingsSimKinRel(model);

% run simulation

data = mystica.runSimKinRel('model',model,'stgs',stgs,'mBodyPosQuat_0',model.getMBodyPosQuatRestConfiguration,'nameControllerClass','mystica.controller.ExampleKinRel');
% mystica.controller.ExampleKinRel is a controller that generates random values of motorsAngVel

% visualize data

if stgs.visualizer.run
    mystica.viz.visualizeKinRel('model',model,'data',data,'stgs',stgs)
end
