function updatePlot(obj,k)

    if obj.stgsVisualizer.livePerformances.run && isempty(obj.dataLiveStatistics)==0
        obj.plotLiveStatistics('indexStart',1,'indexEnd',k,'phase','update')
    end

    mBodyPosQuat_0 = obj.data.mBodyPosQuat_0(:,k);

    for i = 1 : obj.model.nLink
        linkPosQuat_0 = mBodyPosQuat_0(obj.model.linksAttributes{i}.selector.indexes_linkPosQuat_from_mBodyPosQuat);
        tform_0_b     = mystica.rbm.getTformGivenPosQuat(linkPosQuat_0);

        % CSYS
        if obj.stgsVisualizer.mBody.bodyCSYS.show
            obj.plotCSYS(tform_0_b,obj.stgsVisualizer.mBody.bodyCSYS.dim,'NOprepareFigure',obj.structLink{i}.bodyCSYS);
        end
        if obj.stgsVisualizer.mBody.jointCSYS.show
            for j = 1 : length(obj.model.linksAttributes{i}.tform_b_j)
                obj.plotCSYS(tform_0_b*obj.model.linksAttributes{i}.tform_b_j{j},obj.stgsVisualizer.mBody.jointCSYS.dim,'NOprepareFigure',obj.structLink{i}.jointCSYS{j});
            end
        end

        % MESH
        vertices = obj.getCoordinatesVerticesSTL('tform_0_originSTL',tform_0_b,'vertices',obj.structLink{i}.bodySTLvertices,'scale',obj.model.linksAttributes{i}.visual.scale);
        set(obj.structLink{i}.bodySTL,'Vertices',vertices);

        % NORMALS
        if ( obj.stgsVisualizer.desiredShape.normal.show || obj.stgsVisualizer.desiredShape.points.show ) && obj.stgsVisualizer.desiredShape.fun.update
            posXeff  = tform_0_b(1,4);
            posYeff  = tform_0_b(2,4);
            posZdes  = obj.stgsDesiredShape.fun(posXeff,posYeff,obj.data.time(k));
            if obj.stgsVisualizer.desiredShape.normal.show
                normalDirection = full(obj.functionDesiredNormal(mystica.rbm.getPosGivenTform(tform_0_b),obj.data.time(k)));
                normalSegment = [posXeff;posYeff;posZdes] + obj.stgsVisualizer.desiredShape.normal.dim * [zeros(3,1) normalDirection];
                set(obj.structDesiredShape.normal{i},'XData',normalSegment(1,:),'YData',normalSegment(2,:),'ZData',normalSegment(3,:));
            end
            if obj.stgsVisualizer.desiredShape.points.show
                set(obj.structDesiredShape.points{i},'XData',posXeff,'YData',posYeff,'ZData',posZdes);
            end
        end

    end

    % SURFACE
    if obj.stgsVisualizer.desiredShape.fun.show && obj.stgsVisualizer.desiredShape.fun.update
        obj.structDesiredShape.fun.Z = obj.stgsDesiredShape.fun(obj.structDesiredShape.fun.X,obj.structDesiredShape.fun.Y,obj.data.time(k));
        set(obj.structDesiredShape.fun.surf,'XData',obj.structDesiredShape.fun.X,'YData',obj.structDesiredShape.fun.Y,'ZData',obj.structDesiredShape.fun.Z);
    end

    % JOINTS
    for j = 1 : obj.model.nJoint
        if obj.stgsVisualizer.joint.cone.show
            set(obj.structJoint{j}.cone,'Vertices',obj.getConeVertices(mBodyPosQuat_0,j));
        end
        if obj.stgsVisualizer.joint.sphere.show && (sum(obj.model.joints{j}.axesActuated) || obj.stgsVisualizer.joint.sphere.showNAct)
            set(obj.structJoint{j}.sphere,'Vertices',obj.getSphereVertices(mBodyPosQuat_0,j));
        end
    end

end
