function runVisualizer(obj,input)
    arguments
        obj
        input.indexesVisualizer  = obj.indexesVisualizer;
        input.timeInterval (2,1) = [inf,inf];
        input.pauseDelay   (1,1) = 0;
    end

    if all(input.timeInterval == inf)
        obj.indexesVisualizer = input.indexesVisualizer;
        if any(obj.indexesVisualizer > length(obj.data.time))
            obj.indexesVisualizer(obj.indexesVisualizer > length(obj.data.time)) = length(obj.data.time);
        end
    else
        obj.stgsIntegrator.maxTimeStep
        [~,idx1] = min(abs(obj.data.time-input.timeInterval(1)));
        [~,idx2] = min(abs(obj.data.time-input.timeInterval(2)));
        obj.indexesVisualizer = [idx1:idx2];
    end
    % Remove duplicates
    obj.indexesVisualizer([false diff(obj.indexesVisualizer)==0]) = [];

    nFramesMBodySimulation = length(obj.indexesVisualizer);
    nFramesInitialRotation = obj.stgsVisualizer.cameraView.initialRotation.run*obj.stgsVisualizer.cameraView.initialRotation.durationTotal*obj.stgsVisualizer.frameRate;
    nFramesFinalRotation   = obj.stgsVisualizer.cameraView.finalRotation.run  *obj.stgsVisualizer.cameraView.finalRotation.durationTotal  *obj.stgsVisualizer.frameRate;

    figureFrames(nFramesMBodySimulation+nFramesInitialRotation+nFramesFinalRotation) = getframe(obj.figure);
    obj.figureFrames = figureFrames;


    %% Initial Rotation

    if obj.stgsVisualizer.cameraView.initialRotation.run
        obj.rotateCameraView(obj.stgsVisualizer.cameraView.initialRotation.durationTotal,...
                             obj.stgsVisualizer.cameraView.initialRotation.pause.start,...
                             obj.stgsVisualizer.cameraView.initialRotation.pause.end,...
                             obj.stgsVisualizer.cameraView.initialRotation.values,...
                             obj.stgsVisualizer.cameraView.mBodySimulation.values,...
                             0)
    end

    %% MBodySimulation

    counterFrames = 0 + nFramesInitialRotation;
    for k = obj.indexesVisualizer

        obj.indexIterationVis = k;

        counterFrames = counterFrames + 1;
        obj.updatePlot(obj.indexIterationVis)
        pause(input.pauseDelay)

        if obj.stgsVisualizer.gif.save || obj.stgsVisualizer.video.save
            obj.figureFrames(counterFrames) = getframe(obj.figure);
        end

        if any(obj.indexIterationVis == obj.indexesStatusTrackerPrint)
            %clc
            fprintf('Visualization Time: %.1f/%.0f\n',obj.data.time(obj.indexIterationVis),obj.stgsVisualizer.limitMaximumTime);
        end

    end

    %% Final Rotation

    if obj.stgsVisualizer.cameraView.finalRotation.run
        obj.rotateCameraView(obj.stgsVisualizer.cameraView.finalRotation.durationTotal,...
                             obj.stgsVisualizer.cameraView.finalRotation.pause.start,...
                             obj.stgsVisualizer.cameraView.finalRotation.pause.end,...        
                             obj.stgsVisualizer.cameraView.mBodySimulation.values,...
                             obj.stgsVisualizer.cameraView.finalRotation.values,...
                             counterFrames)
    end

end
