function color = getSphereColor(obj,j)
    
    R = [0.8500; 0.3250; 0.0980];
    G = [0.4660; 0.6740; 0.1880];
    B = [0     ; 0.4470; 0.7410];
    
    colorMatrix = [R G B];
    
    rotm_b_j = mystica.rbm.getRotmGivenTform(obj.model.linksAttributes{obj.model.joints{j}.linkParent}.tform_b_j{obj.model.joints{j}.connectionPointParent});
    
    if all(obj.model.joints{j}.axesActuated==0)
        color = obj.stgsVisualizer.joint.sphere.colorNAct;
    else
        if obj.stgsVisualizer.joint.sphere.colorBodyFrame
            color = colorMatrix * abs(rotm_b_j*obj.model.joints{j}.axesActuated);
        else
            color = colorMatrix * obj.model.joints{j}.axesActuated;
        end
    end
    
    if max(color)>1
        color = color/max(color);
    end
    
end
