function structPlot = plotDataSummary(input)
    arguments
        input.time (1,:)
        input.dir  = 1;
        input.Data
        input.summaryMethod = 'mean'
        input.rangeMethod   = 'std'
        input.prctileValues = []
    end

    if input.dir == 1
        M = input.Data;
    else
        M = transpose(input.Data);
    end

    switch input.summaryMethod
        case 'mean'
            summary = mean(M);
    end

    switch input.rangeMethod
        case 'std'
            lowerBound = mean(M) - std(M);
            upperBound = mean(M) + std(M);
        case 'prctile'
            lowerBound = prctile(M,input.prctileValues(1));
            upperBound = prctile(M,input.prctileValues(2));
    end

    hold on
    structPlot.summary = plot(input.time,summary,'LineWidth',1.5);

    if strcmp(input.rangeMethod,'none') == 0
        structPlot.range = fill([input.time' ; flipud(input.time')],[upperBound' ; flipud(lowerBound')], get(structPlot.summary,'Color'));
        set(structPlot.range,'FaceAlpha',0.2)
        set(structPlot.range,'LineStyle','none')
    end
    hold off

    structPlot.stgs.dir = input.dir;
    structPlot.stgs.summaryMethod = input.summaryMethod;
    structPlot.stgs.rangeMethod = input.rangeMethod;
    structPlot.stgs.prctileValues = input.prctileValues;

end
