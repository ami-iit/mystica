function visual = visualizeDynRel(input)
    arguments
        input.model
        input.data
        input.stgs
    end
    dataLiveStatistics{1} = struct('data',abs(input.data.errorOrientationNormals),'title','Error alignment normal vectors','ylabel','Angle $[deg]$');
    dataLiveStatistics{2} = struct('data',abs(input.data.errorPositionNormals),'title','Node position error','ylabel','Distance $[m]$');
    dataLiveStatistics{3} = struct('data',abs(input.data.motorsCurrent),'title','Motor Current','ylabel','Current $[A]$');
    visual = mystica.viz.VisualizerMatlab('data',input.data,'stgs',input.stgs,'model',input.model,'dataLiveStatistics',dataLiveStatistics);
    visual.runVisualizer()
    visual.save()
end
