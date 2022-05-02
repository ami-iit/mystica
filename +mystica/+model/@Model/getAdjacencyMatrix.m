function getAdjacencyMatrix(obj,cellAdjacency)
    n = size(cellAdjacency,1);
    m = size(cellAdjacency,2);
    if n~=m
        error('adjacency Matrix is not square')
    end
    adjacencyMatrix = zeros(n);
    for i = 1 : n
        for j = 1 : n
            adjacencyMatrix(i,j) = (isempty(cellAdjacency{i,j}) == 0);
        end
    end
    obj.adjacencyMatrix = sparse(adjacencyMatrix);
end
