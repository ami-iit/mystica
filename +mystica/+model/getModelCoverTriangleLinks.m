function model = getModelCoverTriangleLinks(input)
    arguments
        input.n
        input.m
        input.restConfiguration {mustBeMember(input.restConfiguration,{'flat','cylinder'})}
        input.linkDimension %[m]
    end

    [umc,meshDesign.unitMeas]=mystica.model.setUM('length','m','mass','kg');

    meshDesign.shape = 'triangle';
    meshDesign.baseIndex = 1;
    meshDesign.fixedLinksIndexes = [meshDesign.baseIndex];
    meshDesign.tform_0_bBase = mystica.rbm.getTformGivenPosRotm(zeros(3,1)*(umc.length),mystica.rbm.getRotmGivenEul('rz',pi/3)); %[m]*(umc.length)
    meshDesign.linkDimension = input.linkDimension*(umc.length); %[m]*(umc.length)

    input.name = [meshDesign.shape,sprintf('_%ix%i',input.n,input.m)];

    %%

    cellModel = mystica.model.CellModel('nameModel',input.name,'unitMeas',meshDesign.unitMeas);

    switch input.restConfiguration
        case 'cylinder'
            cellModel.createMeshTriangleNodePatternRectangle(...
                'n',input.n,...
                'm',input.m,...
                'linkDimension',meshDesign.linkDimension,...
                'cylinder',true);
        case 'flat'
            cellModel.createMeshTriangleNodePatternRectangle(...
                'n',input.n,...
                'm',input.m,...
                'linkDimension',meshDesign.linkDimension,...
                'cylinder',false);
    end

    % Visual
    cellModel.assignLinkProperty('name','stlScale'    ,'value',meshDesign.linkDimension)
    cellModel.assignLinkProperty('name','stlName'     ,'value',fullfile('+mystica','+viz','meshes','nodeSimplifiedTriangle.stl'))
    cellModel.assignLinkProperty('name','stlFaceColor','value',[0.5,0.7,0.9],'indexes',1:2:length(cellModel.cellLinks))
    cellModel.assignLinkProperty('name','stlFaceColor','value',[0.5,0.7,0.3],'indexes',2:2:length(cellModel.cellLinks))
    cellModel.assignLinkProperty('name','stlFaceColor','value',[0.00,0.40,0.40],'indexes',meshDesign.fixedLinksIndexes)
    cellModel.assignLinkProperty('name','stlEdgeColor','value','k')

    % Geometry
    cellModel.assignLinkProperty('name','mass'           ,'value',0.01*(umc.mass))                 %[kg]    *(umc.mass)
    cellModel.assignLinkProperty('name','inertiaTens_g_g','value',eye(3)*(umc.mass*umc.length.^2)) %[kg*m^2]*(umc.mass*umc.length.^2)
    cellModel.assignLinkProperty('name','tform_b_g'      ,'value',eye(4))

    cellModel.assignLinkProperty('name','tform_0_b'      ,'value',zeros(4))   % [m]
    cellModel.assignLinkProperty('name','tform_0_b'      ,'value',meshDesign.tform_0_bBase,'indexes',meshDesign.baseIndex) % [m]

    cellModel.assignLinkLogicalProperty('name','baseLink','value',1,'indexes',meshDesign.baseIndex)
    cellModel.assignLinkLogicalProperty('name','fixed'   ,'value',1,'indexes',meshDesign.fixedLinksIndexes)

    % Joint
    cellModel.assignJointProperty('name','limitRoM'        ,'value',50*pi/180)                  %[rad]
    cellModel.assignJointProperty('name','limitJointVel'   ,'value',20*pi/180)                  %[rad/s]
    cellModel.assignJointProperty('name','limitJointTorque','value',7*(umc.mass*umc.length.^2)) %[Nm]*(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('name','jointType'       ,'value','revolute')                 % [char]
    cellModel.assignJointProperty('name','axesRotation'    ,'value',[0 0 1])                    % (3,1)
    cellModel.assignJointProperty('name','axesActuated'    ,'value',[0 0 0])                    % (3,1)

    cellModel.assignJointProperty('name','coeffViscousFriction'   ,'value',0*(umc.mass*umc.length.^2)) %[kg*m^2/(rad*s)]*(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('name','coeffCoulombFriction'   ,'value',1*(umc.mass*umc.length.^2)) %[Nm]            *(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('name','coeffMotorTorque'       ,'value',0*(umc.mass*umc.length.^2)) %[Nm/A]          *(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('name','inertiaTensMotorRot_j_g','value',0*(umc.mass*umc.length.^2)) %[kg*m^2]        *(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('name','transmissionGearRatio'  ,'value',0)                          %[]
    cellModel.assignJointProperty('name','transmissionEfficiency' ,'value',0)                          %[]

    %% getModel

    model = mystica.model.Model(cellModel);

end
