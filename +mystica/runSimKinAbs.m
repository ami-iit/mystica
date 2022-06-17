function [data,stateKin] = runSimKinAbs(input)
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

    stateKin = mystica.state.StateKinMBody(...
        'model',model,...
        'mBodyPosQuat_0',input.mBodyPosQuat_0,...
        'stgsStateKinMBody',stgs.stateKin);

    contr = ClassController(...
        'model',model,...
        'stateKinMBody',stateKin,...
        'stgsController',stgs.controller,....
        'stgsDesiredShape',stgs.desiredShape,...
        'time',0,...
        'controller_dt',stgs.integrator.maxTimeStep);

    intgr = mystica.intgr.IntegratorKinAbs(...
        'dt',stgs.integrator.maxTimeStep,...
        'stgsIntegrator',stgs.integrator);

    data  = mystica.log.LoggerKinAbs(...
        'model',model,...
        'numberIterations',length(kVec));

    %% Simulation

    for k = kVec
        % Controller
        mBodyTwist_0 = contr.solve('time',tout(k),'stateKinMBody',stateKin,'model',model);
        % Integrator
        mBodyPosQuat_0 = intgr.integrate('stateKinMBody',stateKin,'mBodyTwist_0',mBodyTwist_0,'model',model);
        % Logger
        data.store('indexIteration',k,'time',tout(k),'model',model,'controller',contr,'stateKinMBody',stateKin,'stgsDesiredShape',stgs.desiredShape,'mBodyTwist_0',mBodyTwist_0);
        % New State
        stateKin.setMBodyPosQuat('mBodyPosQuat_0',mBodyPosQuat_0,'model',model);
    end

    %% Saving Workspace

    clear ans k kVec mBodyTwist_0 mBodyPosQuat_0 tout
    mystica.utils.deleteGeneratedMEX

    if stgs.saving.workspace.run
        if stgs.saving.workspace.clearCasadi
            contr.clearProperties();
            stateKin.clearProperties();
        end
        save(stgs.saving.workspace.name)
    end

end
