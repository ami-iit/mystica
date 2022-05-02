function createMeshTriangleNodePatternRectangle(obj,mesh)
    arguments
        obj
        mesh.n
        mesh.m
        mesh.linkDimension
        mesh.cylinder logical = false
    end
    
    %
    %      /\
    %   1 /  \ 2
    %    /____\
    %       3
    
    nLinks = 2*mesh.n*mesh.m;
    
    G = reshape(1:2:nLinks,mesh.m,mesh.n)';
    obj.initialize('nLinks',nLinks);
    
    if mesh.cylinder
        [angleH,angleV] = obj.getYoshimuraBucklingDimensions(60,mesh.m);
    else
        angleH = 0;
        angleV = 0;
    end
    
    for k = 1 : nLinks
        
        
        obj.assignLinkProperty('indexes',k,'name','linkDimension','value',mesh.linkDimension)
        obj.assignLinkProperty('indexes',k,'name','tform_b_j'    ,'value',createFramesTriangleNode(obj.cellLinks{k}.linkDimension))
        
        i = ceil(k/(2*mesh.m));
        j = ceil((mod(k,2*mesh.m)+2*mesh.m*(mod(k,2*mesh.m)==0))/2);
        
        if mod(i,2) == 0
            if mod(k,2) == 0
                if j<mesh.m
                    indexParent = G(i,j)+1;
                    indexChild  = G(i,j+1);
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexParent','value',indexParent)
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexChild' ,'value',indexChild )
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','connectionPoint','value',...
                        struct('parent',3,'child',3,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1),mystica.rbm.getRotmGivenEul('ry',pi,'rz',angleH))))
                end
            else
                indexParent = G(i,j);
                indexChild  = G(i,j)+1;
                obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexParent','value',indexParent)
                obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexChild' ,'value',indexChild )
                obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','connectionPoint','value',...
                    struct('parent',1,'child',1,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1),mystica.rbm.getRotmGivenEul('ry',pi,'rz',angleH))))
                if i<mesh.n
                    indexParent = G(i,j);
                    indexChild  = G(i+1,j);
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexParent','value',indexParent)
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexChild' ,'value',indexChild )
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','connectionPoint','value',...
                        struct('parent',2,'child',2,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1),mystica.rbm.getRotmGivenEul('ry',pi,'rz',angleV))))
                end
            end
        else
            if mod(k,2) == 0
                if i<mesh.n
                    indexParent = G(i,j)+1;
                    indexChild  = G(i+1,j)+1;
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexParent','value',indexParent)
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexChild' ,'value',indexChild )
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','connectionPoint','value',...
                        struct('parent',2,'child',2,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1),mystica.rbm.getRotmGivenEul('ry',pi,'rz',angleV))))
                end
                if j<mesh.m
                    indexParent = G(i,j)+1;
                    indexChild  = G(i,j+1);
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexParent','value',indexParent)
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexChild' ,'value',indexChild )
                    obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','connectionPoint','value',...
                        struct('parent',1,'child',1,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1),mystica.rbm.getRotmGivenEul('ry',pi,'rz',angleH))))
                end
            else
                indexParent = G(i,j);
                indexChild  = G(i,j)+1;
                obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexParent','value',indexParent)
                obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexChild' ,'value',indexChild )
                obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','connectionPoint','value',...
                    struct('parent',3,'child',3,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1),mystica.rbm.getRotmGivenEul('ry',pi,'rz',angleH))))
            end
        end
        
        
    end
    
end

function tform_b_j = createFramesTriangleNode(edgeLength)
    %       /\
    %    1 /  \ 2
    %     /____\
    %        3
    e = 0.5*tan(30*pi/180);
    tform_b_j{1} = mystica.rbm.getTformGivenPosRotm([ cos(150*pi/180) ; sin(150*pi/180) ; 0]*e*edgeLength,mystica.rbm.getRotmGivenEul('rz',pi-pi/6,'rx',pi/2));
    tform_b_j{2} = mystica.rbm.getTformGivenPosRotm([ cos( 30*pi/180) ; sin( 30*pi/180) ; 0]*e*edgeLength,mystica.rbm.getRotmGivenEul('rz',pi/6,'rx',pi/2));
    tform_b_j{3} = mystica.rbm.getTformGivenPosRotm([ cos(270*pi/180) ; sin(270*pi/180) ; 0]*e*edgeLength,mystica.rbm.getRotmGivenEul('ry',-pi/2,'rz',-pi/2));
end
