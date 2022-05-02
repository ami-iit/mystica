classdef NullSpace < handle
    
    properties (SetAccess = protected, GetAccess = public)
        A
        Z
        rank = []
        singValues
        stgs
        condA
    end
    properties (SetAccess = protected, GetAccess = protected)
        linIndRow
        V
    end
    
    methods
        function obj = NullSpace(input)
            arguments
                input.decompositionMethod    char   = 'svd'
                input.rankRevealingMethod    char   = 'limitSingularValue_tolAuto'
                input.toleranceRankRevealing double = []
                input.A                      double = []
            end
            obj.stgs.decompositionMethod   = input.decompositionMethod;
            obj.stgs.rankRevealingMethod   = input.rankRevealingMethod;
            obj.stgs.toleranceRankRevealing = input.toleranceRankRevealing;
            
            if isempty(input.A) == 0
                obj.compute(input.A)
            end
        end
        
        function Z = compute(obj,A)
            switch obj.stgs.decompositionMethod
                case 'qrFull'
                    obj.A = full(A);
                    obj.qr();
                case 'qrSparse'
                    obj.A = sparse(A);
                    obj.qr();
                case 'svd'
                    obj.A = full(A);
                    obj.svd();
            end
            Z = obj.Z;
        end
        
        function linIndRow = getLinIndRow(obj)
            switch obj.stgs.decompositionMethod
                case 'svd'
                    [~,~,p] = qr(transpose(obj.A),'vector');
                    obj.linIndRow = sort(p(1:obj.rank));
            end
            linIndRow = obj.linIndRow;
        end
        
    end
    
    methods (Access=protected)
        getRank(obj)
        
        function qr(obj)
            [Q,R,p] = qr(transpose(obj.A),'vector');
            obj.singValues = full(abs(diag(R)));
            obj.V = full(Q);
            obj.getRank();
            obj.Z = obj.V(:,(obj.rank+1):end);
            obj.linIndRow = sort(p(1:obj.rank));
        end
        
        function svd(obj)
            [~,S,obj.V] = svd(obj.A,0);
            obj.singValues = diag(S);
            obj.getRank();
            obj.Z = obj.V(:,(obj.rank+1):end);
        end
        
    end
    
end
