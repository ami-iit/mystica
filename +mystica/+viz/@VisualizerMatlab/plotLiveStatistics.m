function plotLiveStatistics(obj,input)
    arguments
        obj
    end
    arguments
        input.indexStart
        input.indexEnd
        input.phase
    end

    nRows = length(obj.dataLiveStatistics);
    nColu = 4;
    matrix = reshape(1 : nRows*nColu,nColu,nRows)';

    switch input.phase
        case 'create'
            for i = 1 : nRows
                subplot(nRows,nColu,matrix(i,nColu))
                obj.structPlotStatistics{i} = mystica.utils.plotDataSummary(...
                    'Data',obj.dataLiveStatistics{i}.data(:,input.indexStart:input.indexEnd),...
                    'time',obj.data.time(input.indexStart:input.indexEnd),...
                    'rangeMethod','prctile',...
                    'prctileValues',obj.stgsVisualizer.livePerformances.prctileValues);

                xlim(obj.data.time([obj.indexesVisualizer(1) obj.indexesVisualizer(end)]))
                if min(obj.dataLiveStatistics{i}.data(:)) ~= max(obj.dataLiveStatistics{i}.data(:))
                    ylim([min(obj.dataLiveStatistics{i}.data(:)) max(obj.dataLiveStatistics{i}.data(:))])
                end
                ylabel(obj.dataLiveStatistics{i}.ylabel,'Interpreter','LaTex')
                xlabel('Time $[s]$','Interpreter','LaTex')
                title(obj.dataLiveStatistics{i}.title,'Interpreter','LaTex')
            end
            subplot(nRows,nColu,matrix(:,1:nColu-1))
        case 'update'
            for i = 1 : nRows
                subplot(nRows,nColu,matrix(i,nColu))
                mystica.utils.plotDataSummaryUpdate(...
                    'Data',obj.dataLiveStatistics{i}.data(:,input.indexStart:input.indexEnd),...
                    'time',obj.data.time(input.indexStart:input.indexEnd),...
                    'structPlot',obj.structPlotStatistics{i});
            end
            subplot(nRows,nColu,matrix(:,1:nColu-1))
    end
end
