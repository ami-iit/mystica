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
        pinvAred
        pinvA
        V
    end

    methods
        function obj = NullSpace(input)
            arguments
                input.decompositionMethod    char {mustBeMember(input.decompositionMethod,{'svd','qrFull','qrSparse'})} = 'svd'
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

        function pinvAred = getPinvAred(obj)
            switch obj.stgs.decompositionMethod
                case 'svd'
                    [Q,R,P] = qr(transpose(obj.A));
                    obj.pinvAred = obj.computePinvQR(Q,R,P);
            end
            pinvAred = obj.pinvAred;
        end

        function pinvA = getPinvA(obj)
            switch obj.stgs.decompositionMethod
                case {'qrFull','qrSparse'}
                    [U,S,V] = svd(obj.A,0);
                    obj.pinvA = obj.computePinvSVD(U,S,V);
            end
            pinvA = obj.pinvA;
        end

    end

    methods (Access=protected)
        getRank(obj)

        function qr(obj)
            [Q,R,P] = qr(transpose(obj.A));
            [p,~] = find(P); p=p';
            obj.singValues = full(abs(diag(R)));
            obj.V = full(Q);
            obj.getRank();
            obj.Z = obj.V(:,(obj.rank+1):end);
            obj.linIndRow = sort(p(1:obj.rank));
            %
            obj.pinvAred = obj.computePinvQR(Q,R,P);
        end

        function svd(obj)
            [U,S,obj.V] = svd(obj.A,0);
            obj.singValues = diag(S);
            obj.getRank();
            obj.Z = obj.V(:,(obj.rank+1):end);
            %
            obj.pinvA = obj.computePinvSVD(U,S,obj.V);
        end

        function pinvAred = computePinvQR(obj,Q,R,P)
            Q1 = Q(:,1:obj.rank);
            R1 = R(1:obj.rank,1:obj.rank);
            P1 = P(:,1:obj.rank); P1(sum(P1,2)==0,:)=[];
            pinvAred = Q1*((R1')\(inv(P1)));
        end

        function pinvA = computePinvSVD(obj,U,S,V)
            Ur = U(:,1:obj.rank);
            Sr = S(1:obj.rank,1:obj.rank); invSr = diag(1./diag(Sr));
            Vr = V(:,1:obj.rank);
            pinvA = Vr * invSr * Ur';
        end

    end

end
