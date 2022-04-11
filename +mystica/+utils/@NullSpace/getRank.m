function getRank(obj)
    
    s = [obj.singValues;0];
    V = [obj.V,zeros(size(obj.V,1),1)];
    
    switch obj.stgs.rankRevealingMethod
        case 'limitSingularValue_tolAuto'
            obj.stgs.toleranceRankRevealing = max(size(obj.A)) * eps(max(s));
            indexFirstZero = find(s < obj.stgs.toleranceRankRevealing,1);
        case 'limitSingularValue'
            indexFirstZero = find(s < obj.stgs.toleranceRankRevealing,1);
        case 'limitRatioSingularValues'
            indexFirstZero = find( s./[s(1); s(1:end-1)] <  obj.stgs.toleranceRankRevealing,1);
        case 'limitParFunRatioSingularValues'
            indexFirstZero = find( (s./[s(1); s(1:end-1)]).^mystica.utils.logB('x',s(1)./s,'base',obj.stgs.toleranceRankRevealing(1)) <  obj.stgs.toleranceRankRevealing(2),1);
        case 'limitConditionNumber'
            indexFirstZero = find( s(1)./s >  obj.stgs.toleranceRankRevealing,1);
        case 'limitErrorNullSpace'
            N = obj.A*V/norm(obj.A,'fro');
            indexFirstZero = find( flip(sqrt(cumsum(flip(sum(N.^2,1))))) < obj.stgs.toleranceRankRevealing,1);
    end
    
    if obj.rank ~= indexFirstZero-1
        warning('rank changed')
    end
    
    obj.rank = indexFirstZero-1;
    obj.condA = obj.singValues(1)/obj.singValues(obj.rank);
    
end
