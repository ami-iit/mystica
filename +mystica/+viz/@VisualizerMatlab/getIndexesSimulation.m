function getIndexesSimulation(obj,input)
    arguments
        obj
        input.stgsIntegrator
    end

    timeCut = obj.data.time(obj.data.time<=obj.stgsVisualizer.limitMaximumTime);

    Tintegrator = input.stgsIntegrator.maxTimeStep;
    Tvisualizer = 1/obj.stgsVisualizer.frameRate;
    TtimePrint  = 1/obj.stgsVisualizer.statusTracker.workspacePrint.frameRate;

    if Tvisualizer/Tintegrator < 2
        Tvisualizer = Tintegrator;
        obj.stgsVisualizer.frameRate = 1/Tvisualizer;
        obj.indexesVisualizer = 1 : length(timeCut);
    else
        posPeaks = find(islocalmax(-mod(timeCut,Tvisualizer)));
        obj.indexesVisualizer = [1  posPeaks  length(timeCut)];
    end

    if TtimePrint/Tvisualizer < 2
        TtimePrint = Tvisualizer;
        obj.stgsVisualizer.statusTracker.workspacePrint.frameRate = 1/TtimePrint;
        obj.indexesStatusTrackerPrint = obj.indexesVisualizer;
    else
        posPeaks = find(islocalmax(-mod(timeCut(obj.indexesVisualizer),TtimePrint)));
        posPeaks = [1  posPeaks length(obj.indexesVisualizer)];
        obj.indexesStatusTrackerPrint = obj.indexesVisualizer(posPeaks);
    end

end
