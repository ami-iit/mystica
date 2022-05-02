function vertices = getCoordinatesVerticesSTL(input)
    arguments
        input.tform_0_originSTL
        input.vertices
        input.scale    (1,3)
    end

    vertices = ones(size(input.vertices,1),4);
    vertices(:,1:3) = input.vertices.*input.scale;
    vertices = transpose(input.tform_0_originSTL * transpose(vertices));
    vertices = vertices(:,1:3);

end
