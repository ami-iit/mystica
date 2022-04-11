function model = getModel4BarLinkage()

    [umc,meshDesign.unitMeas]=mystica.model.setUM('length','m','mass','kg');

    meshDesign.shape = '4BarLinkage';
    meshDesign.baseIndex = 1;
    meshDesign.fixedLinksIndexes = [meshDesign.baseIndex];
    meshDesign.tform_0_bBase = mystica.rbm.getTformGivenPosRotm(zeros(3,1)*(umc.length),mystica.rbm.getRotmGivenEul('rz',0)); %[m]*(umc.length)

    meshDesign.name = [meshDesign.shape];

    %% Initialization

    cellModel = mystica.model.CellModel('nameModel',meshDesign.name,'unitMeas',meshDesign.unitMeas);
    cellModel.initialize('nLinks',4)

    %-------------------------------------------------------------------------%
    % common properties
    %
    % Geometry
    cellModel.assignLinkProperty('name','tform_0_b','value',zeros(4))   % [m]
    cellModel.assignLinkProperty('name','baseLink' ,'value',0)
    cellModel.assignLinkProperty('name','fixed'    ,'value',0)
    % Visual
    cellModel.assignLinkProperty('name','stlName'     ,'value',fullfile('+mystica','+viz','meshes','cube.stl'))
    cellModel.assignLinkProperty('name','stlEdgeColor','value','none')

    %-------------------------------------------------------------------------%
    % Link Fixed & Base
    %
    % Geometry
    cellModel.assignLinkProperty('name','tform_0_b','value',meshDesign.tform_0_bBase,'indexes',meshDesign.baseIndex) %[m]*(umc.length)
    cellModel.assignLinkProperty('name','baseLink' ,'value',1                       ,'indexes',meshDesign.baseIndex)
    cellModel.assignLinkProperty('name','fixed'    ,'value',1                       ,'indexes',meshDesign.fixedLinksIndexes)
    % Visual
    cellModel.assignLinkProperty('name','stlFaceColor','value',[0.00,0.40,0.40]     ,'indexes',meshDesign.fixedLinksIndexes)

    %% Link1

    % Geometry
    cellModel.assignLinkProperty('indexes',1,'name','tform_b_g'      ,'value',eye(4))
    cellModel.assignLinkProperty('indexes',1,'name','tform_b_j'      ,'value',{...
        mystica.rbm.getTformGivenPosRotm([0;-0.250;0.025]*(umc.length),mystica.rbm.getRotmGivenEul('ry',-pi/2,'rz',-pi/2))
        mystica.rbm.getTformGivenPosRotm([0;+0.250;0.025]*(umc.length),mystica.rbm.getRotmGivenEul('ry',+pi/2,'rz',+pi/2))})       %[m] *(umc.length)
    cellModel.assignLinkProperty('indexes',1,'name','mass'           ,'value',3.499999999999996*(umc.mass)) %[kg]*(umc.mass)
    cellModel.assignLinkProperty('indexes',1,'name','inertiaTens_g_g','value',diag([73645.83333333327 1458.3333333333303 73645.83333333327])/1e6*(umc.mass*umc.length.^2)) %[kg*m^2]*(umc.mass*umc.length.^2)
    % Visual
    cellModel.assignLinkProperty('indexes',1,'name','linkDimension'  ,'value',0.5*(umc.length)) %[m]*(umc.length)
    cellModel.assignLinkProperty('indexes',1,'name','stlScale'       ,'value',[0.05 0.5 0.05]*(umc.length))
    cellModel.assignLinkProperty('indexes',1,'name','stlFaceColor'   ,'value',[0.04 0.43 0.28])

    %% Link2

    % Geometry
    cellModel.assignLinkProperty('indexes',2,'name','tform_b_g'      ,'value',eye(4))
    cellModel.assignLinkProperty('indexes',2,'name','tform_b_j'      ,'value',{...
        mystica.rbm.getTformGivenPosRotm([0;-0.100;0.025]*(umc.length),mystica.rbm.getRotmGivenEul('ry',-pi/2,'rz',-pi/2))
        mystica.rbm.getTformGivenPosRotm([0;+0.100;0.025]*(umc.length),mystica.rbm.getRotmGivenEul('ry',+pi/2,'rz',+pi/2))})        %[m] *(umc.length)
    cellModel.assignLinkProperty('indexes',2,'name','mass'           ,'value',1.3999999999999995*(umc.mass)) %[kg]*(umc.mass)
    cellModel.assignLinkProperty('indexes',2,'name','inertiaTens_g_g','value',diag([4958.3333333333321 583.33333333333303 4958.3333333333321])/1e6*(umc.mass*umc.length.^2)) %[kg*m^2]*(umc.mass*umc.length.^2)
    % Visual
    cellModel.assignLinkProperty('indexes',2,'name','linkDimension'  ,'value',0.2*(umc.length)) %[m]*(umc.length)
    cellModel.assignLinkProperty('indexes',2,'name','stlScale'       ,'value',[0.05 0.2 0.05]*(umc.length))
    cellModel.assignLinkProperty('indexes',2,'name','stlFaceColor'   ,'value',[0.44 0.67 0.66])

    %% Link3

    % Geometry
    cellModel.assignLinkProperty('indexes',3,'name','tform_b_g'      ,'value',eye(4))
    cellModel.assignLinkProperty('indexes',3,'name','tform_b_j'      ,'value',{...
        mystica.rbm.getTformGivenPosRotm([0;-0.150;0.025]*(umc.length),mystica.rbm.getRotmGivenEul('ry',-pi/2,'rz',-pi/2))
        mystica.rbm.getTformGivenPosRotm([0;+0.150;0.025]*(umc.length),mystica.rbm.getRotmGivenEul('ry',+pi/2,'rz',+pi/2))})        %[m] *(umc.length)
    cellModel.assignLinkProperty('indexes',3,'name','mass'           ,'value',2.0999999999999996*(umc.mass)) %[kg]*(umc.mass)
    cellModel.assignLinkProperty('indexes',3,'name','inertiaTens_g_g','value',diag([16187.499999999995 874.99999999999966 16187.499999999995])/1e6*(umc.mass*umc.length.^2)) %[kg*m^2]*(umc.mass*umc.length.^2)
    % Visual
    cellModel.assignLinkProperty('indexes',3,'name','linkDimension'  ,'value',0.3*(umc.length)) %[m]*(umc.length)
    cellModel.assignLinkProperty('indexes',3,'name','stlScale'       ,'value',[0.05 0.3 0.05]*(umc.length))
    cellModel.assignLinkProperty('indexes',3,'name','stlFaceColor'   ,'value',[0.15 0.67 0.59])

    %% Link4

    % Geometry
    cellModel.assignLinkProperty('indexes',4,'name','tform_b_g'      ,'value',eye(4))
    cellModel.assignLinkProperty('indexes',4,'name','tform_b_j'      ,'value',{...
        mystica.rbm.getTformGivenPosRotm([0;-0.200;0.025]*(umc.length),mystica.rbm.getRotmGivenEul('ry',-pi/2,'rz',-pi/2))
        mystica.rbm.getTformGivenPosRotm([0;+0.200;0.025]*(umc.length),mystica.rbm.getRotmGivenEul('ry',+pi/2,'rz',+pi/2))})        %[m] *(umc.length)
    cellModel.assignLinkProperty('indexes',4,'name','mass'           ,'value',2.7999999999999989*(umc.mass)) %[kg]*(umc.mass)
    cellModel.assignLinkProperty('indexes',4,'name','inertiaTens_g_g','value',diag([37916.666666666657 1166.6666666666661 37916.666666666657])/1e6*(umc.mass*umc.length.^2)) %[kg*m^2]*(umc.mass*umc.length.^2)
    % Visual
    cellModel.assignLinkProperty('indexes',4,'name','linkDimension'  ,'value',0.4*(umc.length)) %[m]*(umc.length)
    cellModel.assignLinkProperty('indexes',4,'name','stlScale'       ,'value',[0.05 0.4 0.05]*(umc.length))
    cellModel.assignLinkProperty('indexes',4,'name','stlFaceColor'   ,'value',[0.29 0.65 0.48])

    %% Joint 1(P)-2(C)

    cellModel.assignJointPropertyGivenFamily('link1',1,'link2',2,'name','indexParent'    ,'value',1)
    cellModel.assignJointPropertyGivenFamily('link1',1,'link2',2,'name','indexChild'     ,'value',2)
    cellModel.assignJointPropertyGivenFamily('link1',1,'link2',2,'name','connectionPoint','value',...
        struct('parent',2,'child',1,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1)*(umc.length),mystica.rbm.getRotmGivenEul('ry',pi,'rz',-59.783880831764982*pi/180))))

    %% Joint 2(P)-3(C)

    cellModel.assignJointPropertyGivenFamily('link1',2,'link2',3,'name','indexParent'    ,'value',2)
    cellModel.assignJointPropertyGivenFamily('link1',2,'link2',3,'name','indexChild'     ,'value',3)
    cellModel.assignJointPropertyGivenFamily('link1',2,'link2',3,'name','connectionPoint','value',...
        struct('parent',2,'child',1,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1)*(umc.length),mystica.rbm.getRotmGivenEul('ry',pi,'rz',-105.03284228495727*pi/180))))

    %% Joint 3(P)-4(C)

    cellModel.assignJointPropertyGivenFamily('link1',3,'link2',4,'name','indexParent'    ,'value',3)
    cellModel.assignJointPropertyGivenFamily('link1',3,'link2',4,'name','indexChild'     ,'value',4)
    cellModel.assignJointPropertyGivenFamily('link1',3,'link2',4,'name','connectionPoint','value',...
        struct('parent',2,'child',1,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1)*(umc.length),mystica.rbm.getRotmGivenEul('ry',pi,'rz',-54.122616400106402*pi/180))))

    %% Joint 4(P)-1(C)

    cellModel.assignJointPropertyGivenFamily('link1',4,'link2',1,'name','indexParent'    ,'value',4)
    cellModel.assignJointPropertyGivenFamily('link1',4,'link2',1,'name','indexChild'     ,'value',1)
    cellModel.assignJointPropertyGivenFamily('link1',4,'link2',1,'name','connectionPoint','value',...
        struct('parent',2,'child',1,'tform_pj0_cj0',mystica.rbm.getTformGivenPosRotm(zeros(3,1)*(umc.length),mystica.rbm.getRotmGivenEul('ry',pi,'rz',-141.06066048317138*pi/180))))

    %% Joint Common Properties

    mask_active  = zeros(4,4); mask_active(sub2ind([4,4],[1],[2]))          = 1;
    mask_passive = zeros(4,4); mask_passive(sub2ind([4,4],[2,3,4],[3,4,1])) = 1;

    cellModel.assignJointProperty('name','limitRoM'        ,'value',50*pi/180)                  %[rad]
    cellModel.assignJointProperty('name','limitJointVel'   ,'value',20*pi/180)                  %[rad/s]
    cellModel.assignJointProperty('name','limitJointTorque','value',7*(umc.mass*umc.length.^2)) %[Nm]*(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('name','jointType'       ,'value','revolute')                 %[char]
    cellModel.assignJointProperty('name','axesRotation'    ,'value',[0 0 1])                    %(3,1)

    % Define axes actuated
    cellModel.assignJointProperty('mask',mask_active ,'name','axesActuated','value',[0 0 1])    %(3,1)
    cellModel.assignJointProperty('mask',mask_passive,'name','axesActuated','value',[0 0 0])    %(3,1)
    % Actuated axes of rotation
    cellModel.assignJointProperty('mask',mask_active,'name','coeffViscousFriction'   ,'value',0.5*(umc.mass*umc.length.^2)) %[kg*m^2/(rad*s)]*(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('mask',mask_active,'name','coeffCoulombFriction'   ,'value',0  *(umc.mass*umc.length.^2)) %[Nm]            *(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('mask',mask_active,'name','coeffMotorTorque'       ,'value',9  *(umc.mass*umc.length.^2)) %[Nm/A]          *(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('mask',mask_active,'name','inertiaTensMotorRot_j_g','value',0  *(umc.mass*umc.length.^2)) %[kg*m^2]        *(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('mask',mask_active,'name','transmissionGearRatio'  ,'value',1)
    cellModel.assignJointProperty('mask',mask_active,'name','transmissionEfficiency' ,'value',1)
    % Passive axes of rotation
    cellModel.assignJointProperty('mask',mask_passive,'name','coeffViscousFriction'  ,'value',0.5*(umc.mass*umc.length.^2)) %[kg*m^2/(rad*s)]*(umc.mass*umc.length.^2)
    cellModel.assignJointProperty('mask',mask_passive,'name','coeffCoulombFriction'  ,'value',0  *(umc.mass*umc.length.^2)) %[Nm]            *(umc.mass*umc.length.^2)

    %% getModel

    model = mystica.model.Model(cellModel);

end
