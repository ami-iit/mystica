function nameNew = createTimeTrackerFile(nameOld,baseName,actualValue,maxValue)

    arguments
        nameOld char
        baseName char
        actualValue double
        maxValue double = 0;
    end

    if isempty(nameOld) ~= 1
        delete(nameOld)
    end
    if maxValue == 0
        nameNew = [baseName,'_',num2str(actualValue),'.txt'];
        fileID = fopen(nameNew,'w');
        fclose(fileID);
    elseif actualValue < maxValue
        nameNew = [baseName,'_',num2str(actualValue),'-',num2str(maxValue),'.txt'];
        fileID = fopen(nameNew,'w');
        fclose(fileID);
    else
        nameNew = '';
    end

end
