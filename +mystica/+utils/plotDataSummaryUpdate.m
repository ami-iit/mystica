function structPlot = plotDataSummaryUpdate(input)
    arguments
        input.time (1,:)
        input.Data
        input.structPlot
    end

    if input.structPlot.stgs.dir == 1
        M = input.Data;
    else
        M = transpose(input.Data);
    end

    switch input.structPlot.stgs.summaryMethod
        case 'mean'
            if all(size(M)>1)
                summary = mean(M);
            else
                summary = M;
            end
    end

    switch input.structPlot.stgs.rangeMethod
        case 'std'
            lowerBound = mean(M) - std(M);
            upperBound = mean(M) + std(M);
        case 'prctile'
            lowerBound = prctile(M,input.structPlot.stgs.prctileValues(1));
            upperBound = prctile(M,input.structPlot.stgs.prctileValues(2));
    end


    set(input.structPlot.summary,'xdata',input.time,'ydata',summary);

    if strcmp(input.structPlot.stgs.rangeMethod,'none') == 0
        set(input.structPlot.range,'xdata',[input.time' ; flipud(input.time')],'ydata',[upperBound' ; flipud(lowerBound')]);
    end

end
