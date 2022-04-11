function saveF(nameFull,Dim)
    %
    % function saveF(nameFull,Dim)
    %
    % saveF('name.ext',[dim1,dim2])
    %
    % 'name'        - file name
    % 'ext'         - is the file extension, it could be 'png','eps' or 'pdf'
    % [dim1,dim2]   - dimensions in centimeters [width,height]

    % Dimensions
    width=Dim(1);
    height=Dim(2);

    % Name
    pointerP=find(nameFull=='.');
    name=nameFull(1:pointerP-1);
    extension=nameFull(pointerP+1:end);


    % Paper settings
    set(gcf,'InvertHardcopy','on'); % background color
    set(gcf,'PaperUnits', 'centimeters'); % unit of measurament

    papersize = get(gcf, 'PaperSize'); % get the paper size
    left = (papersize(1)- width)/2;
    bottom = (papersize(2)- height)/2;
    myfiguresize = [left, bottom, width, height];
    set(gcf,'PaperPosition', myfiguresize); % set the paper position

    % Save
    switch extension
        case 'png'
            print(gcf,name,'-dpng','-r300');
        case 'eps'
            print(gcf,name,'-depsc2','-r300');
        case 'pdf'
            set(gcf,'PaperSize',[width height]);
            print(gcf,name,'-dpdf','-r300','-fillpage');
    end

end
