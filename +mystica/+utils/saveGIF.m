function saveGIF(filename,frames,frameRate,compressionRatio)

    if nargin < 4
        compressionRatio = 1;
    end


    numberQuantizedColors = 256;

    if isempty(filename) == 1
        error('Define a proper file name')
    end

    if nargin == 2
        delayTime = 0;
        % A value of 0 displays images as fast as your hardware allows.
    else
        delayTime = 1/frameRate*compressionRatio;
    end

    disp('GIF | Processing')
    for i=1:compressionRatio:length(frames)

        % set figure background to white
        set(gcf,'color','w');

        imRGB = frame2im(frames(i));
        [imIND,cm] = rgb2ind(imRGB,numberQuantizedColors);

        % Write to the GIF File
        if i == 1
            imwrite(imIND,cm,filename,'gif','Loopcount',inf,'DelayTime',delayTime);
        else
            imwrite(imIND,cm,filename,'gif','WriteMode','append','DelayTime',delayTime);
        end
    end
    disp('GIF | Created')

end
