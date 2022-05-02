function dataOut = merge(obj,dataIn,stgs)
    arguments
        obj
    end
    arguments (Repeating)
        dataIn mystica.log.Logger
    end
    arguments
        stgs.boolFlip logical = zeros(length(dataIn),1)
        stgs.fixTime  logical = 0
    end

    dataOut = copy(obj);
    names = fieldnames(dataOut);

    for j = 1 : length(names)
        dataOut.(names{j}) = [];
    end

    for i = 1 : length(dataIn)
        for j = 1 : length(names)
            dataOut.(names{j}) = [dataOut.(names{j}) ...
                dataIn{i}.(names{j})*not(stgs.boolFlip(i)) + flip(dataIn{i}.(names{j}),2)*stgs.boolFlip(i)];
        end
    end

    if stgs.fixTime
        dataOut.time = (0 : length(dataOut.time)-1)*obj.time(2);
    end

end
