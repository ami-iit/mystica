function createPlot(obj,stgsUM)

    obj.indexIterationVis = 1;
    if obj.stgsVisualizer.livePerformances.run && isempty(obj.dataLiveStatistics)==0
        obj.plotLiveStatistics('indexStart',obj.indexIterationVis,'indexEnd',obj.indexIterationVis,'phase','create')
    end

    hold on
    obj.plotCSYS(eye(4),obj.stgsVisualizer.origin.dimCSYS,'prepareFigure')
    xlabel(sprintf('x $[%s]$',stgsUM.length),'Interpreter','LaTex')
    ylabel(sprintf('y $[%s]$',stgsUM.length),'Interpreter','LaTex')
    zlabel(sprintf('z $[%s]$',stgsUM.length),'Interpreter','LaTex')
    mBodyPosQuat_0 = obj.data.mBodyPosQuat_0(:,obj.indexIterationVis);
    obj.structLink{obj.model.nLink} = [];

    if obj.stgsVisualizer.figure.showAxis == 0
        axis off
    end

    % backgroundVisualizer
    if isempty(obj.stgsVisualizer.background)==0
        for i = 1 : length(obj.stgsVisualizer.background)
            triangulation = stlread(obj.stgsVisualizer.background{i}.stlName);
            faces    = triangulation.ConnectivityList;
            vertices = triangulation.Points;
            vertices = obj.getCoordinatesVerticesSTL('tform_0_originSTL',obj.stgsVisualizer.background{i}.tform_0_originSTL,'vertices',vertices,'scale',obj.stgsVisualizer.background{i}.scale);
            patch('Faces',faces,'Vertices',vertices,'FaceColor',obj.stgsVisualizer.background{i}.FaceColor,'EdgeColor',obj.stgsVisualizer.background{i}.EdgeColor,'FaceAlpha',obj.stgsVisualizer.background{i}.FaceAlpha,'LineWidth',0.1,'FaceLighting','gouraud','AmbientStrength',0.25);
        end
    end

    % Multibody
    for i = 1 : obj.model.nLink
        linkPosQuat_0 = mBodyPosQuat_0(obj.model.linksAttributes{i}.selector.indexes_linkPosQuat_from_mBodyPosQuat);
        tform_0_b     = mystica.rbm.getTformGivenPosQuat(linkPosQuat_0);

        % CSYS
        if obj.stgsVisualizer.mBody.bodyCSYS.show
            obj.structLink{i}.bodyCSYS = obj.plotCSYS(tform_0_b,obj.stgsVisualizer.mBody.bodyCSYS.dim,'NOprepareFigure');
        end
        if obj.stgsVisualizer.mBody.jointCSYS.show
            for j = 1 : length(obj.model.linksAttributes{i}.tform_b_j)
                obj.structLink{i}.jointCSYS{j} = obj.plotCSYS(tform_0_b*obj.model.linksAttributes{i}.tform_b_j{j},obj.stgsVisualizer.mBody.jointCSYS.dim,'NOprepareFigure');
            end
        end

        % MESH
        triangulation = stlread(obj.model.linksAttributes{i}.visual.name);
        faces = triangulation.ConnectivityList;
        vertices = triangulation.Points;
        obj.structLink{i}.bodySTLvertices = vertices;
        vertices = obj.getCoordinatesVerticesSTL('tform_0_originSTL',tform_0_b,'vertices',vertices,'scale',obj.model.linksAttributes{i}.visual.scale);
        obj.structLink{i}.bodySTL = patch('Faces',faces,...
            'Vertices',vertices,...
            'FaceColor',obj.model.linksAttributes{i}.visual.faceColor,...
            'EdgeColor',obj.model.linksAttributes{i}.visual.edgeColor,...
            'LineWidth',0.1,...
            'FaceLighting','gouraud',...
            'AmbientStrength',0.25);

        % NORMALS
        if obj.stgsVisualizer.desiredShape.normal.show || obj.stgsVisualizer.desiredShape.points.show
            posXeff  = tform_0_b(1,4);
            posYeff  = tform_0_b(2,4);
            posZdes  = obj.stgsDesiredShape.fun(posXeff,posYeff,obj.data.time(obj.indexIterationVis));
            if obj.stgsVisualizer.desiredShape.normal.show
                normalDirection = full(obj.functionDesiredNormal(mystica.rbm.getPosGivenTform(tform_0_b),obj.data.time(obj.indexIterationVis)));
                normalSegment = [posXeff;posYeff;posZdes] + obj.stgsVisualizer.desiredShape.normal.dim * [zeros(3,1) normalDirection];
                obj.structDesiredShape.normal{i} = line(normalSegment(1,:),normalSegment(2,:),normalSegment(3,:),...
                    'linewidth',obj.stgsVisualizer.desiredShape.normal.linewidth,...
                    'color',obj.stgsVisualizer.desiredShape.normal.color);
            end
            if obj.stgsVisualizer.desiredShape.points.show
                obj.structDesiredShape.points{i}= line(posXeff,posYeff,posZdes,...
                    'Marker',obj.stgsVisualizer.desiredShape.points.markerSym,...
                    'MarkerSize',obj.stgsVisualizer.desiredShape.points.markerSize,...
                    'color',obj.stgsVisualizer.desiredShape.points.color,...
                    'MarkerFaceColor',obj.stgsVisualizer.desiredShape.points.colorFace);
            end
        end

    end

    % SURFACE
    if obj.stgsVisualizer.desiredShape.fun.show
        xPlotFun = obj.figureMatrixLimits(1,1) : obj.model.linksAttributes{1}.linkDimension/10 : obj.figureMatrixLimits(1,2);
        yPlotFun = obj.figureMatrixLimits(2,1) : obj.model.linksAttributes{1}.linkDimension/10 : obj.figureMatrixLimits(2,2);
        [obj.structDesiredShape.fun.X,obj.structDesiredShape.fun.Y] = meshgrid(xPlotFun,yPlotFun);

        obj.structDesiredShape.fun.Z = obj.stgsDesiredShape.fun(obj.structDesiredShape.fun.X,obj.structDesiredShape.fun.Y,obj.data.time(obj.indexIterationVis));

        obj.structDesiredShape.fun.surf = surf(obj.structDesiredShape.fun.X,...
            obj.structDesiredShape.fun.Y,...
            obj.structDesiredShape.fun.Z,...
            'FaceAlpha',obj.stgsVisualizer.desiredShape.fun.faceAlpha,...
            'EdgeAlpha',obj.stgsVisualizer.desiredShape.fun.edgeAlpha,...
            'EdgeColor',obj.stgsVisualizer.desiredShape.fun.edgeColor,...
            'FaceColor',obj.stgsVisualizer.desiredShape.fun.faceColor);
    end

    % JOINTS
    if obj.stgsVisualizer.joint.cone.show
        triangulation = stlread(obj.stgsVisualizer.joint.cone.stlName);
        obj.structJointShared.coneGeometry.faces    = triangulation.ConnectivityList;
        obj.structJointShared.coneGeometry.vertices = triangulation.Points;
    end
    if obj.stgsVisualizer.joint.sphere.show
        triangulation = stlread(obj.stgsVisualizer.joint.sphere.stlName);
        obj.structJointShared.sphereGeometry.faces    = triangulation.ConnectivityList;
        obj.structJointShared.sphereGeometry.vertices = triangulation.Points;
    end
    for j = 1 : obj.model.nJoint
        if obj.stgsVisualizer.joint.cone.show
            obj.structJoint{j}.cone = patch('Faces',obj.structJointShared.coneGeometry.faces,...
                'Vertices',obj.getConeVertices(mBodyPosQuat_0,j),...
                'FaceColor',obj.stgsVisualizer.joint.cone.color,...
                'FaceAlpha',obj.stgsVisualizer.joint.cone.faceAlpha,...
                'EdgeColor','none',...
                'LineWidth',0.1,...
                'FaceLighting','gouraud',...
                'AmbientStrength',0.25);
        end
        if obj.stgsVisualizer.joint.sphere.show && (sum(obj.model.joints{j}.axesActuated) || obj.stgsVisualizer.joint.sphere.showNAct)
            obj.structJoint{j}.sphere = patch('Faces',obj.structJointShared.sphereGeometry.faces,...
                'Vertices',obj.getSphereVertices(mBodyPosQuat_0,j),...
                'FaceColor',obj.getSphereColor(j),...
                'FaceAlpha',obj.stgsVisualizer.joint.sphere.faceAlpha,...
                'EdgeColor','none',...
                'LineWidth',0.1,...
                'FaceLighting','gouraud',...
                'AmbientStrength',0.25);
        end
    end

    hold off
end
