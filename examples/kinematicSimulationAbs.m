%% kinematic Simulation controlling mBodyTwist_0

clear
clc
close all
fclose('all');

% generate Model

switch 'cylinder_triangleLinks'
    case 'cylinder_triangleLinks'
        model = mystica.model.getModelCoverTriangleLinks('n',3,'m',5,'restConfiguration','cylinder','linkDimension',0.0425);
    case 'flat_triangleLinks'
        model = mystica.model.getModelCoverTriangleLinks('n',3,'m',5,'restConfiguration','flat','linkDimension',0.0425);
    case 'flat_squareLinks'
        model = mystica.model.getModelCoverSquareLinks('n',3,'m',5,'restConfiguration','flat','linkDimension',0.0425);
    case '4barLinkage'
        model = mystica.model.getModel4BarLinkage();
end

% define simulation settings and parameters

stgs  = mystica.stgs.getDefaultSettingsSimKinAbs(model);

% run simulation

data  = mystica.runSimKinAbs('model',model,'stgs',stgs,'mBodyPosQuat_0',model.getMBodyPosQuatRestConfiguration,'nameControllerClass','mystica.controller.ExampleKinAbs');
% mystica.controller.ExampleKinAbs is a controller that generates random values of mBodyTwist_0

% visualize data

if stgs.visualizer.run
    mystica.viz.visualizeKinAbs('model',model,'data',data,'stgs',stgs);
end
