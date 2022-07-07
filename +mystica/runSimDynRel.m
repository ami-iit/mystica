function [data,stateDyn] = runSimDynRel(input)
    arguments
        input.stgs                 struct
        input.model                mystica.model.Model
        input.mBodyPosQuat_0 (:,1) double
        input.nameControllerClass  char
    end

    stgs            = input.stgs;
    model           = input.model;
    ClassController = str2func(input.nameControllerClass);

    %% Initialization

    tout  = 0 : stgs.integrator.maxTimeStep : stgs.integrator.limitMaximumTime;
    kVec  = 1 : length(tout);

    stateDyn = mystica.state.StateDynMBody(...
        'model',model,...
        'mBodyPosQuat_0',input.mBodyPosQuat_0,...
        'mBodyTwist_0',zeros(model.constants.mBodyTwist,1),...
        'stgsIntegrator',stgs.integrator,...
        'stgsStateDynMBody',stgs.stateDyn,...
        'stgsModel',stgs.model);

    contr = ClassController(...
        'model',model,...
        'stateDynMBody',stateDyn,...
        'stgsController',stgs.controller,....
        'stgsDesiredShape',stgs.desiredShape,...
        'time',0,...
        'controller_dt',stgs.integrator.maxTimeStep);

    intgr = mystica.intgr.IntegratorDynRel(...
        'dt',stgs.integrator.maxTimeStep,...
        'stgsIntegrator',stgs.integrator);

    data  = mystica.log.LoggerDynRel(...
        'model',model,...
        'stateDynMBody',stateDyn,...
        'numberIterations',length(kVec),...
        'stgsIntegrator',stgs.integrator);

    %% Integration

    for k = kVec
        % Controller
        motorsCurrent = contr.solve('stateDynMBody',stateDyn,'time',tout(k),'model',model) * stgs.controller.applyControlInput;
        % Integrator
        mBodyPosVel_0 = intgr.integrate('stateDynMBody',stateDyn,'motorsCurrent',motorsCurrent,'model',model);
        % Logger
        data.store('indexIteration',k,'time',tout(k),'model',model,'controller',contr,'stateDynMBody',stateDyn,'motorsCurrent',motorsCurrent,...
            'stgsDesiredShape',stgs.desiredShape)
        % New State
        stateDyn.setMBodyPosVel('mBodyPosVel_0',mBodyPosVel_0,'model',model);
    end

    %% Saving Workspace

    clear ans k kVec motorsCurrent mBodyPosQuat_0 tout dataLiveStatistics

    if stgs.saving.workspace.run
        if stgs.saving.workspace.clearCasadi
            contr.clearProperties();
            stateDyn.clearProperties();
        end
        save(stgs.saving.workspace.name)
    end

end
