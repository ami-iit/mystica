function plot(obj)
    
    s = [obj.singValues;0];
    V = [obj.V,zeros(size(obj.V,1),1)];

    figure
    sgtitle(sprintf('DoF: %i | norm(AZ,''fro''): %.2e',size(obj.Z,2),norm(obj.A*obj.Z)))
    
    subplot(5,1,1)
    fun = s;
    plotValue(obj.rank,fun,'singular values')
    if strcmp(obj.stgs.rankRevealingMethod,'limitSingularValue') || strcmp(obj.stgs.rankRevealingMethod,'limitSingularValue_tolAuto')
        plot([1 length(fun)],ones(2,1)*obj.stgs.toleranceRankRevealing,'--g')
    end
    
    subplot(5,1,2)
    fun = s./[s(1); s(1:end-1)];
    plotValue(obj.rank,fun,'ratio singular values')
    if strcmp(obj.stgs.rankRevealingMethod,'limitRatioSingularValues')
        plot([1 length(fun)],ones(2,1)*obj.stgs.toleranceRankRevealing,'--g')
    end
    
    subplot(5,1,3)
    fun = s(1)./s;
    plotValue(obj.rank,fun,'condition number')
    if strcmp(obj.stgs.rankRevealingMethod,'limitConditionNumber')
        plot([1 length(fun)],ones(2,1)*obj.stgs.toleranceRankRevealing,'--g')
    end
    
    subplot(5,1,4)
    if strcmp(obj.stgs.rankRevealingMethod,'limitParFunRatioSingularValues')
        tol = obj.stgs.toleranceRankRevealing(1);
    else
        tol = 10;
    end
    fun = mystica.utils.saturateSignal((s./[s(1); s(1:end-1)]).^mystica.utils.logB('x',s(1)./s,'base',tol),inf,1e-20);
    plotValue(obj.rank,fun,'parametrized function')
    if strcmp(obj.stgs.rankRevealingMethod,'limitParFunRatioSingularValues')
        plot([1 length(fun)],ones(2,1)*obj.stgs.toleranceRankRevealing(2),'--g')
    end
    
    subplot(5,1,5)
    N = obj.A*V/norm(obj.A,'fro');
    fun = mystica.utils.saturateSignal(flip(sqrt(cumsum(flip(sum(N.^2,1))))),inf,1e-20);
    plotValue(obj.rank,fun,'error nullspace')
    if strcmp(obj.stgs.rankRevealingMethod,'limitErrorNullSpace')
        plot([1 length(fun)],ones(2,1)*obj.stgs.toleranceRankRevealing,'--g')
    end
    
end

%%

function plotValue(rk,y,yLabelString)
    hold on
    plot(y,'o-')
    plot(rk  ,y(rk  ),'*b')
    plot(rk+1,y(rk+1),'*r')
    set(gca,'yscale','log')
    axis tight
    ylimHold = ylim;
    plot(rk+0.5*ones(2,1),ylimHold,'--r')
    grid on
    title(sprintf('Last Accepted %.2e | First Discarged %.2e',y(rk),y(rk+1)))
    xlabel('#singular value')
    ylabel(yLabelString)
    xlim([1 length(y)]);
    ylim(ylimHold);
end
