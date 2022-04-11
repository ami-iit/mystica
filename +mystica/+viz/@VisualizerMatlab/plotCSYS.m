function varargout = plotCSYS(H,scaleLengthCSYS,prepareFigure,structPlot)

    % Create or update plot?

    if nargin == 4
        caseChoice = 'updatePlot';
    else
        caseChoice = 'createPlot';
    end

    %% Compute CSYS values

    posOriginNode = H(1:3,4);
    R_W_0  = H(1:3,1:3);

    % R
    xR = posOriginNode(1)+scaleLengthCSYS*[0 R_W_0(1,1)];
    yR = posOriginNode(2)+scaleLengthCSYS*[0 R_W_0(2,1)];
    zR = posOriginNode(3)+scaleLengthCSYS*[0 R_W_0(3,1)];

    % G
    xG = posOriginNode(1)+scaleLengthCSYS*[0 R_W_0(1,2)];
    yG = posOriginNode(2)+scaleLengthCSYS*[0 R_W_0(2,2)];
    zG = posOriginNode(3)+scaleLengthCSYS*[0 R_W_0(3,2)];

    % B
    xB = posOriginNode(1)+scaleLengthCSYS*[0 R_W_0(1,3)];
    yB = posOriginNode(2)+scaleLengthCSYS*[0 R_W_0(2,3)];
    zB = posOriginNode(3)+scaleLengthCSYS*[0 R_W_0(3,3)];

    %% Plot

    switch caseChoice
        case 'createPlot'
            structPlot.versorR = line(xR,yR,zR,'Color','r','linewidth',3);
            structPlot.versorG = line(xG,yG,zG,'Color','g','linewidth',3);
            structPlot.versorB = line(xB,yB,zB,'Color','b','linewidth',3);
        case 'updatePlot'
            set(structPlot.versorR,'XData',xR,'YData',yR,'ZData',zR);
            set(structPlot.versorG,'XData',xG,'YData',yG,'ZData',zG);
            set(structPlot.versorB,'XData',xB,'YData',yB,'ZData',zB);
    end

    %% Add axis

    if strcmp(prepareFigure,'prepareFigure')
        view(3)
        axis equal
        xlabel('x $[m]$','Interpreter','LaTex')
        ylabel('y $[m]$','Interpreter','LaTex')
        zlabel('z $[m]$','Interpreter','LaTex')
    end

    %% Output

    if nargout > 0
        varargout{1} = structPlot;
    end

end
