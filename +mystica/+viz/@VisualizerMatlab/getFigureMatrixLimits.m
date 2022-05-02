function getFigureMatrixLimits(obj,input)
    arguments
        obj
        input.MatrixMBodyPos_0
    end

    linkDimension = obj.model.linksAttributes{1}.linkDimension;

    x = input.MatrixMBodyPos_0(1:3:end,:);
    y = input.MatrixMBodyPos_0(2:3:end,:);
    z = input.MatrixMBodyPos_0(3:3:end,:);

    obj.figureMatrixLimits = [min(min(x)) max(max(x))
                              min(min(y)) max(max(y))
                              min(min(z)) max(max(z))];

    obj.figureMatrixLimits(:,1) = obj.figureMatrixLimits(:,1) - linkDimension/1.5;
    obj.figureMatrixLimits(:,2) = obj.figureMatrixLimits(:,2) + linkDimension/1.5;

    obj.figureMatrixLimits(3,1) = obj.figureMatrixLimits(3,1) - linkDimension/4;
    obj.figureMatrixLimits(3,2) = obj.figureMatrixLimits(3,2) + linkDimension/4;

end
