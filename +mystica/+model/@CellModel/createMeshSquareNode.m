function createMeshSquareNode(obj,mesh)
    arguments
        obj
        mesh.n
        mesh.m
        mesh.linkDimension
        mesh.rzi_pj0_cj0
        mesh.rzj_pj0_cj0
    end
    
    %      1
    %     _|_
    % 4 -|_ _|- 2
    %      |
    %      3
    
    nLinks = mesh.n*mesh.m;
    
    G = reshape(1 : mesh.n*mesh.m,mesh.m,mesh.n)';
    obj.initialize('nLinks',nLinks);
    
    for k = 1 : nLinks
        
        j = (mod(k,mesh.m)==0)*mesh.m + mod(k,mesh.m);
        i = ceil(k/mesh.m);
        
        
        obj.assignLinkProperty('indexes',k,'name','linkDimension','value',mesh.linkDimension)
        obj.assignLinkProperty('indexes',k,'name','tform_b_j'    ,'value',createFramesSquareNode(obj.cellLinks{k}.linkDimension))
        
        indexParent = G(i,j);
        if i<mesh.n
            indexChild  = G(i+1,j);
            obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexParent','value',indexParent)
            obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexChild' ,'value',indexChild )
            obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','connectionPoint','value',...
                struct('parent',3,'child',1,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1),mystica.rbm.getRotmGivenEul('ry',pi,'rz',mesh.rzi_pj0_cj0))))
        end
        if j<mesh.m
            indexChild  = G(i,j+1);
            obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexParent','value',indexParent)
            obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','indexChild' ,'value',indexChild )
            obj.assignJointPropertyGivenFamily('link1',indexParent,'link2',indexChild,'name','connectionPoint','value',...
                struct('parent',2,'child',4,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1),mystica.rbm.getRotmGivenEul('ry',pi,'rz',mesh.rzj_pj0_cj0))))
        end
        
    end
    
end

function tform_b_j = createFramesSquareNode(jointsDistance)
    %      1
    %     _|_
    % 4 -|_ _|- 2
    %      |
    %      3
    tform_b_j{1} = mystica.rbm.getTformGivenPosRotm([ 0   ;  0.5 ; 0]*jointsDistance,mystica.rbm.getRotmGivenEul('ry',pi/2,'rz',pi/2));
    tform_b_j{2} = mystica.rbm.getTformGivenPosRotm([ 0.5 ;  0   ; 0]*jointsDistance,mystica.rbm.getRotmGivenEul('rx',pi/2));
    tform_b_j{3} = mystica.rbm.getTformGivenPosRotm([ 0   ; -0.5 ; 0]*jointsDistance,mystica.rbm.getRotmGivenEul('ry',-pi/2,'rz',-pi/2));
    tform_b_j{4} = mystica.rbm.getTformGivenPosRotm([-0.5 ;  0   ; 0]*jointsDistance,mystica.rbm.getRotmGivenEul('rx',-pi/2,'rz',pi));
end
