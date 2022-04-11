function model = getModelCoverSquareLinks(input)
    arguments
        input.n
        input.m
        input.restConfiguration {mustBeMember(input.restConfiguration,{'flat','cylinder'})}
        input.linkDimension %[m]
    end


    [umc,meshDesign.unitMeas]=mystica.model.setUM('length','m','mass','kg');

    meshDesign.shape = 'square';
    meshDesign.baseIndex = 1;
    meshDesign.fixedLinksIndexes = [meshDesign.baseIndex];
    meshDesign.tform_0_bBase = mystica.rbm.getTformGivenPosRotm(zeros(3,1)*(umc.length),mystica.rbm.getRotmGivenEul('rz',0)); %[m]*(umc.length)
    meshDesign.linkDimension = input.linkDimension*(umc.length); %[m]*(umc.length)

    switch input.restConfiguration
        case 'flat'
            meshDesign.rzi_pj0_cj0 = 0;
            meshDesign.rzj_pj0_cj0 = 0;
        case 'cylinder'
            meshDesign.rzi_pj0_cj0 = 2*pi/input.n;
            meshDesign.rzj_pj0_cj0 = 0;
    end

    input.name = [meshDesign.shape,sprintf('_%ix%i',input.n,input.m)];

    %%

    cellModel = mystica.model.CellModel('nameModel',input.name,'unitMeas',meshDesign.unitMeas);

    cellModel.createMeshSquareNode(...
        'n',input.n,...
        'm',input.m,...
        'linkDimension',meshDesign.linkDimension,...
        'rzi_pj0_cj0',meshDesign.rzi_pj0_cj0,...
        'rzj_pj0_cj0',meshDesign.rzj_pj0_cj0);

    % Visual
    cellModel.assignLinkProperty('name','stlScale'    ,'value',meshDesign.linkDimension*[1 1 1])
    cellModel.assignLinkProperty('name','stlName'     ,'value',fullfile(fileparts(mfilename('fullpath')),'..','+viz','meshes','nodeSquare.stl'))
    cellModel.assignLinkProperty('name','stlFaceColor','value',[0.50,0.70,0.90],'indexes',find(mod(reshape(1:input.n*input.m,[],input.n)' - (1:input.n)',2)'==0))
    cellModel.assignLinkProperty('name','stlFaceColor','value',[0.45,0.75,0.85],'indexes',find(mod(reshape(1:input.n*input.m,[],input.n)' - (1:input.n)',2)'==1))
    cellModel.assignLinkProperty('name','stlFaceColor','value',[0.00,0.40,0.40],'indexes',meshDesign.fixedLinksIndexes)
    cellModel.assignLinkProperty('name','stlEdgeColor','value','none')

    % Geometry
    cellModel.assignLinkProperty('name','mass'           ,'value',0.01*(umc.mass))                     %[kg]    *(umc.mass)
    cellModel.assignLinkProperty('name','inertiaTens_g_g','value',eye(3)/1e6*(umc.mass*umc.length.^2)) %[kg*m^2]*(umc.mass*umc.length.^2)
    cellModel.assignLinkProperty('name','tform_b_g'      ,'value',eye(4))

    cellModel.assignLinkProperty('name','tform_0_b'      ,'value',zeros(4))
    cellModel.assignLinkProperty('name','tform_0_b'      ,'value',meshDesign.tform_0_bBase,'indexes',meshDesign.baseIndex)

    cellModel.assignLinkLogicalProperty('name','baseLink','value',1,'indexes',meshDesign.baseIndex)
    cellModel.assignLinkLogicalProperty('name','fixed'   ,'value',1,'indexes',meshDesign.fixedLinksIndexes)

    % Joint
    cellModel.assignJointProperty('name','limitRoM'        ,'value',50*pi/180)                  %[rad]
    cellModel.assignJointProperty('name','limitJointVel'   ,'value',20*pi/180)                  %[rad/s]
    cellModel.assignJointProperty('name','limitJointTorque','value',7*(umc.mass*umc.length.^2)) %[Nm]*(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('name','jointType'       ,'value','spherical')                %[char]
    cellModel.assignJointProperty('name','axesRotation'    ,'value',[1 1 1])                    %(3,1)
    cellModel.assignJointProperty('name','axesActuated'    ,'value',[0 0 0])                    %(3,1)

    cellModel.assignJointProperty('name','coeffViscousFriction'   ,'value',0   *(umc.mass*umc.length.^2)) %[kg*m^2/(rad*s)]*(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('name','coeffCoulombFriction'   ,'value',0   *(umc.mass*umc.length.^2)) %[Nm]            *(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('name','coeffMotorTorque'       ,'value',1.2 *(umc.mass*umc.length.^2)) %[Nm/A]          *(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('name','inertiaTensMotorRot_j_g','value',0.05*(umc.mass*umc.length.^2)) %[kg*m^2]        *(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('name','transmissionGearRatio'  ,'value',12)                            %[]
    cellModel.assignJointProperty('name','transmissionEfficiency' ,'value',0.9)                           %[]

    %% getModel

    model = mystica.model.Model(cellModel);

end
