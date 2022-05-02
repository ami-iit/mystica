function saveVIDEO(filename,frames,frameRate,qualityVideo)

    if nargin < 4
        qualityVideo = 5;
    end

    if isempty(filename) == 1
        error('Define a proper file name')
    end

    positionPoint=find(filename=='.');
    extension=filename(positionPoint+1:end);

    disp('VIDEO | Processing')

    if strcmp(extension,'avi')
        writerObj = VideoWriter(filename);
    elseif strcmp(extension,'mp4')
        writerObj = VideoWriter(filename,'MPEG-4');
    end

    writerObj.Quality = qualityVideo;

    if nargin >= 3
        writerObj.FrameRate = frameRate;
    end

    open(writerObj);

    for i=1:length(frames)
        writeVideo(writerObj, frames(i));
    end

    close(writerObj);
    disp('VIDEO | Created')
end
