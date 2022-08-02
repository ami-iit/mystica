classdef MeasurePerformance
    properties (Access=protected)
        i_tic
        t0_cpu
    end

    methods
        function obj = MeasurePerformance()
            obj.i_tic = tic;
            obj.t0_cpu = cputime;
        end

        function stats = getPerformance(obj)
            stats.cputime = cputime - obj.t0_cpu;
            stats.timer = toc(obj.i_tic);
        end
    end
end
