function rotateCameraView(obj,durationTotal,durationInitialPhase,durationFinalPhase,initialView,finalView,counterFrames)

    nFrames = durationTotal*obj.stgsVisualizer.frameRate;
    tickInitialPause = durationInitialPhase/durationTotal;
    tickFinalPause   = (durationTotal-durationFinalPhase)/durationTotal;

    for indexStepRotation = 1 : nFrames
        tickInstantValue = indexStepRotation/nFrames;
        deltaAngle  = [wrapDegTo180(finalView(1) - initialView(1)) wrapDegTo180(finalView(2) - initialView(2))];
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

function q = wrapDegTo180(q)
    if abs(q)>=360
        q = mod(q,360);
    end
    if q > 180
        q = q - 360;
    end
    if q <= -180
        q = 360 + q;
    end
end
