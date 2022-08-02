function [data,stateKin,stats] = runSimKinRel(input)
    arguments
        input.stgs                 struct
        input.model                mystica.model.Model
        input.mBodyPosQuat_0 (:,1) double
        input.nameControllerClass  char
    end

    mp = mystica.utils.MeasurePerformance();

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

    intgr = mystica.intgr.IntegratorKinRel(...
        'dt',stgs.integrator.maxTimeStep,...
        'stgsIntegrator',stgs.integrator);

    noise = mystica.noise.NoiseKinRel(...
        'stgsNoise',stgs.noise,...
        'controller_dt',stgs.integrator.maxTimeStep,...
        'kBaum',stgs.integrator.dxdtParam.baumgarteIntegralCoefficient,...
        'regTermDampPInv',stgs.integrator.dxdtParam.regTermDampPInv);
    stateKinNoise = noise.createStateKinMBodyNoise('stateKinMBody',stateKin);

    data  = mystica.log.LoggerKinRel(...
        'model',model,...
        'numberIterations',length(kVec));

    %% Simulation

    for k = kVec
        % Controller
        motorsAngVel      = contr.solve('stateKinMBody',stateKinNoise,'time',tout(k),'model',model)  * stgs.controller.applyControlInput;
        motorsAngVelNoise = noise.applyInputCompression('motorsAngVel',motorsAngVel);
        % Integrator
        mBodyPosQuat_0 = intgr.integrate('stateKinMBody',stateKin,'motorsAngVel',motorsAngVelNoise,'model',model);
        % Logger
        data.store('indexIteration',k,'time',tout(k),'model',model,'controller',contr,'stateKinMBody',stateKin,'stateKinMBodyNoise',stateKinNoise,...
            'stgsDesiredShape',stgs.desiredShape,'motorsAngVel',motorsAngVel,'motorsAngVelNoise',motorsAngVelNoise,'regTermDampPInv',stgs.integrator.dxdtParam.regTermDampPInv)
        % New State
        stateKin.setMBodyPosQuat('mBodyPosQuat_0',mBodyPosQuat_0,'model',model);
        stateKinNoise = noise.applyEstimationError('model',model,'stateKinMBody',stateKin);
    end

    stats = mp.getPerformance();

    %% Saving Workspace

    clear ans k kVec motorsAngVel motorsAngVelNoise mBodyPosQuat_0 tout mp

    if stgs.saving.workspace.run
        if stgs.saving.workspace.clearCasadi
            contr.clearProperties();
            stateKin.clearProperties();
        end
        save(stgs.saving.workspace.name)
    end

end
