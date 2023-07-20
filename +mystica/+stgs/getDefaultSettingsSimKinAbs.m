function stgs = getDefaultSettingsSimKinAbs(model,input)
    arguments
        model
        input.startFile = ''
        input.stgs_integrator_limitMaximumTime (1,1) double = 5
    end

    strTime = [datestr(now,'yyyy-mm-dd'),'_h',datestr(now,'HH-MM')];

    stgs.unitMeas  = model.unitMeas;
    stgs.startFile = input.startFile;

    %% Desired shape

    stgs.desiredShape.fun = @(x,y)   -1*(x.^2+y.^2)*5; %[m]
    stgs.desiredShape.fun = @(x,y,t) stgs.desiredShape.fun(x,y); %[m]
    stgs.desiredShape.fun = @(x,y,t) stgs.desiredShape.fun(x,y,t)-stgs.desiredShape.fun(0,0,t); %[m]
    stgs.desiredShape.fun = @(x,y,t) stgs.desiredShape.fun(x/stgs.unitMeas.converter.length,y/stgs.unitMeas.converter.length,t)*stgs.unitMeas.converter.length; %[m]*(umc.length)
    stgs.desiredShape.invertNormals = 1;

    %% Saving & Logger

    stgs.saving.workspace.run         = 0;
    stgs.saving.workspace.name        = ['kinAbs_',model.name,'_',strTime,'.mat'];
    stgs.saving.workspace.clearCasadi = 0;

    %% StateKin Settings

    stgs.stateKin.nullSpace.decompositionMethod    = 'qrFull';
    stgs.stateKin.nullSpace.rankRevealingMethod    = 'limitParFunRatioSingularValues';
    stgs.stateKin.nullSpace.toleranceRankRevealing = [10 1e-5];

    %% Integration Settings

    stgs.integrator.maxTimeStep       = 1e-2;
    stgs.integrator.limitMaximumTime  = input.stgs_integrator_limitMaximumTime;

    stgs.integrator.solverOpts.name    = 'ode45';
    stgs.integrator.solverOpts.RelTol  = 1e-3;
    stgs.integrator.solverOpts.AbsTol  = 1e-6;
    stgs.integrator.solverOpts.MaxStep = [];

    stgs.integrator.dxdtOpts.assumeConstant = 0;
    stgs.integrator.dxdtParam.baumgarteIntegralCoefficient = 5e1;

    stgs.integrator.statusTracker.workspacePrint.run        = 1;
    stgs.integrator.statusTracker.workspacePrint.frameRate  = 10;
    stgs.integrator.statusTracker.timeTrackerFile.run       = 1;
    stgs.integrator.statusTracker.timeTrackerFile.frameRate = 100;               %[Hz]
    stgs.integrator.statusTracker.timeTrackerFile.baseName  = ['kinAbs_',model.name]; %[char]
    stgs.integrator.statusTracker.limitMaximumTime          = stgs.integrator.limitMaximumTime;

    %% Controller

    stgs.controller.applyControlInput = true;
    stgs.controller.casadi.optimizationType = 'conic';
    stgs.controller.casadi.solver           = 'osqp';
    stgs.controller.costFunction.weightTaskOrientation  = 1;
    stgs.controller.costFunction.weightTaskMinVariation = 0;
    stgs.controller.costFunction.gainLinkAngVelStarAligned        = 30;
    stgs.controller.costFunction.gainLinkAngVelStarOpposite       = 100;
    stgs.controller.costFunction.useFeedForwardTermLinkAngVelStar = 1;


    %% Visualization Settings

    stgs.visualizer.run              = 1;
    stgs.visualizer.frameRate        = 20;
    stgs.visualizer.limitMaximumTime = stgs.integrator.limitMaximumTime;

    stgs.visualizer.statusTracker.workspacePrint.run        = 0;
    stgs.visualizer.statusTracker.workspacePrint.frameRate  = 10;

    stgs.visualizer.origin.dimCSYS            = model.linksAttributes{1}.linkDimension/5;
    stgs.visualizer.mBody.bodyCSYS.show       = 0;
    stgs.visualizer.mBody.bodyCSYS.dim        = model.linksAttributes{1}.linkDimension/10;
    stgs.visualizer.mBody.jointCSYS.show      = 0;
    stgs.visualizer.mBody.jointCSYS.dim       = model.linksAttributes{1}.linkDimension/10;

    stgs.visualizer.joint.cone.show      = 0;
    stgs.visualizer.joint.cone.angleIn   = 50*pi/180;
    stgs.visualizer.joint.cone.angleDe   = 10*pi/180;
    stgs.visualizer.joint.cone.color     = 'g';
    stgs.visualizer.joint.cone.faceAlpha = 0.1;
    stgs.visualizer.joint.cone.dim       = model.linksAttributes{1}.linkDimension/6;

    stgs.visualizer.joint.sphere.show           = 0;
    stgs.visualizer.joint.sphere.colorBodyFrame = 0;
    stgs.visualizer.joint.sphere.showNAct       = 0;
    stgs.visualizer.joint.sphere.colorNAct      = [1 1 1];
    stgs.visualizer.joint.sphere.faceAlpha      = 0.5;
    stgs.visualizer.joint.sphere.dimMin         = model.linksAttributes{1}.linkDimension/3;
    stgs.visualizer.joint.sphere.dimMax         = model.linksAttributes{1}.linkDimension/3;

    stgs.visualizer.desiredShape.fun.show          = 1;
    stgs.visualizer.desiredShape.fun.edgeColor     = [0.5 0.7 0.9];
    stgs.visualizer.desiredShape.fun.faceColor     = [0.5 0.7 0.9];
    stgs.visualizer.desiredShape.fun.edgeAlpha     = 0.1;
    stgs.visualizer.desiredShape.fun.faceAlpha     = 0.1;
    stgs.visualizer.desiredShape.fun.update        = 1;
    stgs.visualizer.desiredShape.normal.show       = 1;
    stgs.visualizer.desiredShape.normal.color      = 'b';
    stgs.visualizer.desiredShape.normal.linewidth  = 3;
    stgs.visualizer.desiredShape.normal.dim        = model.linksAttributes{1}.linkDimension/10;
    stgs.visualizer.desiredShape.points.show       = 0;
    stgs.visualizer.desiredShape.points.color      = 'k';
    stgs.visualizer.desiredShape.points.colorFace  = 'k';
    stgs.visualizer.desiredShape.points.markerSize = 10;
    stgs.visualizer.desiredShape.points.markerSym  = 'o';

    stgs.visualizer.figure.backgroundColor = 'white';
    stgs.visualizer.figure.windowState     = 'maximized';
    stgs.visualizer.figure.position        = [617 157 741 782];
    stgs.visualizer.figure.showAxis        = 1;

    stgs.visualizer.livePerformances.run           = 0;
    stgs.visualizer.livePerformances.prctileValues = [10 90];

    stgs.visualizer.cameraView.mBodySimulation.values        = [-37.5,30];
    stgs.visualizer.cameraView.initialRotation.run           = 0;
    stgs.visualizer.cameraView.initialRotation.durationTotal = 3;
    stgs.visualizer.cameraView.initialRotation.pause.start   = 0;
    stgs.visualizer.cameraView.initialRotation.pause.end     = 0;
    stgs.visualizer.cameraView.initialRotation.values        = [0,0];
    stgs.visualizer.cameraView.finalRotation.run             = 0;
    stgs.visualizer.cameraView.finalRotation.durationTotal   = 3;
    stgs.visualizer.cameraView.finalRotation.pause.start     = 0;
    stgs.visualizer.cameraView.finalRotation.pause.end       = 0;
    stgs.visualizer.cameraView.finalRotation.values          = [90,0];

    stgs.visualizer.background = {};

    stgs.visualizer.gif.save             = 0;
    stgs.visualizer.gif.name             = ['kinAbs_',model.name,'_',strTime,'.gif'];
    stgs.visualizer.gif.compressionRatio = 2;

    stgs.visualizer.video.save    = 0;
    stgs.visualizer.video.name    = ['kinAbs_',model.name,'_',strTime,'.mp4']; % .avi or .mp4 (mp4 only windows and macos see https://it.mathworks.com/help/matlab/ref/videowriter.html#d123e1601330)
    stgs.visualizer.video.quality = 5;

end
