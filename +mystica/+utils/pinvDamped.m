function pinvDampA = pinvDamped(A,regDamp)
    %
    % function pinvDampA = pinvDamped(A,regDamp)
    %
    %     DESCRIPTION: the function computes the damped pseudoinverse of matrix A
    %
    %     USAGE: in a matlab script or function
    %
    %     INPUT:  - A [matrix]: [n x m]
    %               input matrix
    %             - regDamp [scalar]
    %               regularization parameter
    %               if regDamp=0, the problem is solved using matlab pinv function
    %
    %     OUTPUT: - pinvDampA [matrix]: [m x n]
    %               damped pseudoinverse of the input matrix
    %
    %     Genoa, November 2020
    %
    %     See also pinv.

    n = size(A,1);
    m = size(A,2);

    if n == m
        matrixType = 'square';
    elseif n > m
        matrixType = 'skinny';
    else
        matrixType = 'fat';
    end

    if regDamp ~= 0
        switch matrixType
            case 'square'
                pinvDampA = ( transpose(A)*A + regDamp*eye(size(A,2)) ) \ transpose(A);
            case 'skinny' %inv(A'*A)*A'
                pinvDampA = ( transpose(A)*A + regDamp*eye(size(A,2)) ) \ transpose(A);
            case 'fat'    %A'*inv(A*A')
                pinvDampA = transpose(A)/( A*transpose(A) + regDamp*eye(size(A,1)) );
        end
    else
        switch matrixType
            case 'square'
                pinvDampA = A\eye(length(A));
            otherwise
                pinvDampA = pinv(A);
        end
    end

end
