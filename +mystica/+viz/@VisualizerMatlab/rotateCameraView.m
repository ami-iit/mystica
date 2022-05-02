function rotateCameraView(obj,durationTotal,durationInitialPhase,durationFinalPhase,initialView,finalView,counterFrames)

    nFrames = durationTotal*obj.stgsVisualizer.frameRate;
    tickInitialPause = durationInitialPhase/durationTotal;
    tickFinalPause   = (durationTotal-durationFinalPhase)/durationTotal;

    for indexStepRotation = 1 : nFrames
        tickInstantValue = indexStepRotation/nFrames;
        deltaAngle  = wrapTo180(finalView - initialView);
        instantView = initialView + computeCompositeTrajectory(deltaAngle,tickInstantValue,tickInitialPause,tickFinalPause);
        view(instantView);
        if obj.stgsVisualizer.gif.save || obj.stgsVisualizer.video.save
            obj.figureFrames(counterFrames+indexStepRotation) = getframe(obj.figure);
        end
        pause(0)
    end

end

function finalPos = computeSinusoidalTrajectory(angleIncrement,tickInstantValue)
    finalPos = 0.5*(1-cos(pi*tickInstantValue))*angleIncrement;
end

function  finalPos = computeCompositeTrajectory(angleIncrement,tickInstantValue,tickInitialPause,tickFinalPause)
    if tickInstantValue < tickInitialPause
        finalPos = 0;
    elseif tickInstantValue >= tickInitialPause && tickInstantValue < tickFinalPause
        finalPos = computeSinusoidalTrajectory(angleIncrement,(tickInstantValue-tickInitialPause)/(tickFinalPause-tickInitialPause));
    else
        finalPos = angleIncrement;
    end
end
