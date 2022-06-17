function setMBodyRestConfiguration(obj)

    indexesLink = 1 : obj.nLink;
    indexesLink(obj.indexBase) = 0;
    while any(indexesLink~=0)
        indexUnknown = find(indexesLink~=0,1);
        kinematicChain = shortestpath(obj.graph,obj.indexBase,indexUnknown);
        indexKnown = kinematicChain(end-1);
        increment = 0;
        while any(indexKnown == indexesLink)
           increment = increment + 1;
           iVector = find(indexesLink~=0,1+increment);
           indexUnknown = iVector(end);
           kinematicChain = shortestpath(obj.graph,obj.indexBase,indexUnknown);
           indexKnown = kinematicChain(end-1);
        end
        indexesLink(indexUnknown) = 0;
        if any(all((obj.matrixParentChild - [indexUnknown indexKnown])==0,2))
            indexLinkParent = indexUnknown;
            indexLinkChild  = indexKnown;
        else
            indexLinkParent = indexKnown;
            indexLinkChild  = indexUnknown;
        end
        indexJoint      = obj.getJointIndex(indexLinkParent,indexLinkChild);
        indexCPknown    = obj.joints{indexJoint}.connectionPointParent*(indexLinkParent==indexKnown) + ...
                          obj.joints{indexJoint}.connectionPointChild *(indexLinkChild==indexKnown);
        indexCPunknown  = obj.joints{indexJoint}.connectionPointParent*(indexLinkParent==indexUnknown) + ...
                          obj.joints{indexJoint}.connectionPointChild *(indexLinkChild==indexUnknown);
        tform_jCPknown_jCPunknown = obj.joints{indexJoint}.tform_pj0_cj0*(indexLinkParent==indexKnown) + ...
            mystica.rbm.getTformInv(obj.joints{indexJoint}.tform_pj0_cj0)*(indexLinkChild==indexKnown);

        tform_0_bUnknown = obj.linksAttributes{indexKnown}.tform_0_b * obj.linksAttributes{indexKnown}.tform_b_j{indexCPknown} * tform_jCPknown_jCPunknown * mystica.rbm.getTformInv(obj.linksAttributes{indexUnknown}.tform_b_j{indexCPunknown});

        obj.linksAttributes{indexUnknown}.tform_0_b = tform_0_bUnknown;

    end

end
