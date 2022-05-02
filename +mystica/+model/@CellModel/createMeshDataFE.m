function varargout = createMeshDataFE(obj,mesh,visual,femData,debug)
    arguments
        obj
        mesh.baseIndex
        mesh.fixedLinksIndexes
        mesh.tform_0_bBase
        visual.stlScaleHeight
        visual.stlName
        visual.stlFaceColorO
        visual.stlFaceColorE
        visual.stlEdgeColor
        femData.links
        femData.vertices
        debug.plot = 0
    end
    
    nLinks    = size(femData.links,1);
    nVertices = size(femData.links,2);
    
    obj.initialize('nLinks',nLinks);
    
    if debug.plot
        figure
        obj.plotCSYS(eye(4),1,'prepareFigure')
    end
    
    %% Links
    
    for k = 1 : nLinks
        
        coorVertices_0 = transpose(femData.vertices(femData.links(k,:),:));
        coorVerticesExtended_0 = [coorVertices_0 coorVertices_0(:,1)];
        coorJoints_0 = coorVertices_0 + diff(coorVerticesExtended_0,1,2)/2;
        
        dir1 = coorJoints_0(:,1)-coorJoints_0(:,3); distD1 = norm(dir1); desVersor1 = dir1/distD1;
        dir2 = coorJoints_0(:,2)-coorJoints_0(:,4); distD2 = norm(dir2); desVersor2 = dir2/distD2;
        desVersorZ = cross(desVersor1,desVersor2);
        
        pos_0_b = mean(coorJoints_0,2);
        
        rotm_0_b1  = mystica.rbm.getRotmGivenTwoVectors([0;0;1],desVersorZ);
        rotm_b1_b  = mystica.rbm.getRotmGivenEul('rz',acos(dot(desVersor1,rotm_0_b1(:,1))));
        rotm_0_b   = rotm_0_b1 * rotm_b1_b;
        if dot(desVersor1,rotm_0_b(:,1)) < 1-1e-5
            rotm_0_b   = rotm_0_b1 * transpose(rotm_b1_b);
        end
        
        if mod(k,2)
            visual.stlFaceColor  = visual.stlFaceColorO;
        else
            visual.stlFaceColor  = visual.stlFaceColorE;
        end
        
        obj.assignLinkProperty('indexes',k,'name','tform_0_b'   ,'value',mystica.rbm.getTformGivenPosRotm(pos_0_b,rotm_0_b))
        obj.assignLinkProperty('indexes',k,'name','baseLink'    ,'value',(k==mesh.baseIndex))
        obj.assignLinkProperty('indexes',k,'name','fixed'       ,'value',any(k==mesh.fixedLinksIndexes))
        obj.assignLinkProperty('indexes',k,'name','stlName'     ,'value',visual.stlName)
        obj.assignLinkProperty('indexes',k,'name','stlFaceColor','value',visual.stlFaceColor)
        obj.assignLinkProperty('indexes',k,'name','stlEdgeColor','value',visual.stlEdgeColor)
        obj.assignLinkProperty('indexes',k,'name','stlScale'    ,'value',[distD1,distD2,visual.stlScaleHeight])
        obj.assignLinkProperty('indexes',k,'name','linkDimension','value',max(distD1,distD2))
        
        for i = 1 : nVertices
            desZ = coorJoints_0(:,i) - coorVerticesExtended_0(:,i);
            desX = coorVerticesExtended_0(:,i) - mystica.rbm.getPosGivenTform(obj.cellLinks{k}.tform_0_b);
            desY = cross(desZ,desX);
            desY = desY/norm(desY);
            rotm_0_j1  = mystica.rbm.getRotmGivenTwoVectors([0;0;1],desZ);
            rotm_j1_j  = mystica.rbm.getRotmGivenEul('rz',acos(dot(desY,rotm_0_j1(:,2))));
            rotm_0_j   = rotm_0_j1 * rotm_j1_j;
            if dot(desY,rotm_0_j(:,2)) < 1-1e-10
                rotm_0_j   = rotm_0_j1 * transpose(rotm_j1_j);
            end
            tform_b_j{i} = mystica.rbm.getTformInv(obj.cellLinks{k}.tform_0_b) * mystica.rbm.getTformGivenPosRotm(coorJoints_0(:,i),rotm_0_j);
        end
        
        obj.assignLinkProperty('indexes',k,'name','tform_b_j','value',tform_b_j)
        
        if debug.plot
            obj.plotCSYS(obj.cellLinks{k}.tform_0_b,3,'noprepareFigure')
            triangulation = stlread(obj.cellLinks{k}.stlName);
            faces = triangulation.ConnectivityList;
            vertices = triangulation.Points;
            vertices = obj.getCoordinatesVerticesSTL('tform_0_originSTL',obj.cellLinks{k}.tform_0_b,'vertices',vertices,'scale',obj.cellLinks{k}.stlScale);
            patch('Faces',faces,'Vertices',vertices,'FaceColor',obj.cellLinks{k}.stlFaceColor,'EdgeColor',obj.cellLinks{k}.stlEdgeColor,'LineWidth',0.1);
            patch('xdata',coorVertices_0(1,:),'ydata',coorVertices_0(2,:),'zdata',coorVertices_0(3,:),'facecolor','none')
            patch('xdata',coorJoints_0(1,:),'ydata',coorJoints_0(2,:),'zdata',coorJoints_0(3,:),'facecolor','none')
            for i = 1 : nVertices
                obj.plotCSYS(obj.cellLinks{k}.tform_0_b * obj.cellLinks{k}.tform_b_j{i},1,'no')
            end
            text(pos_0_b(1),pos_0_b(2),pos_0_b(3),sprintf('%i',k),'Color',[0 0.4470 0.741],'FontSize',20)
        end
        
    end
    
    tform_0old_bBase = obj.cellLinks{mesh.baseIndex}.tform_0_b;
    tform_0new_bBase = mesh.tform_0_bBase;
    tform_0new_0old  = tform_0new_bBase * mystica.rbm.getTformInv(tform_0old_bBase);
    
    % Change inertial frame
    for k = 1 : nLinks
        obj.assignLinkProperty('indexes',k,'name','tform_0_b','value',tform_0new_0old * obj.cellLinks{k}.tform_0_b)
    end
    
    %% Joints
    
    nJoints = 0;
    verticesBinomialComb = nchoosek(1:size(femData.vertices,1),2);
    
    for i = 1 : size(verticesBinomialComb,1)
        verticesCandidates = verticesBinomialComb(i,:);
        boolV = sum(femData.links-verticesCandidates(1) == 0,2) .* sum(femData.links-verticesCandidates(2) == 0,2) == 1;
        linkIndexes = 1:length(boolV);
        linkIndexes(boolV==0)=[];
        if length(linkIndexes) == 2
            nJoints = nJoints + 1;
            
            indexParent = linkIndexes(1);
            indexChild  = linkIndexes(2);
            
            a = sum(femData.links(indexParent,:) == verticesCandidates');
            b = sum(femData.links(indexChild,:)  == verticesCandidates');
            indexConnectionPointParent = find(a + [a(2:end) a(1)] == 2);
            indexConnectionPointChild  = find(b + [b(2:end) b(1)] == 2);
            
            tform_0_pj0 = obj.cellLinks{indexParent}.tform_0_b * obj.cellLinks{indexParent}.tform_b_j{indexConnectionPointParent};
            tform_0_cj0 = obj.cellLinks{indexChild }.tform_0_b * obj.cellLinks{indexChild }.tform_b_j{indexConnectionPointChild };
            tform_pj0_cj0 = mystica.rbm.getTformInv(tform_0_pj0) * tform_0_cj0;
            
            obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexParent','value',indexParent)
            obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexChild' ,'value',indexChild )
            obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','connectionPoint','value',...
                struct('parent',indexConnectionPointParent,'child',indexConnectionPointChild,'tform_pj0_cj0',tform_pj0_cj0))
            
        elseif length(linkIndexes) > 2
            warning('errore')
        end
    end
    
    if nargout >= 1
        varargout{1}.tform_newRef_oldRef = tform_0new_0old;
    end
    
end
