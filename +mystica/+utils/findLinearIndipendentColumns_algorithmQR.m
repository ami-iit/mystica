function [indexesColumns,varargout]=findLinearIndipendentColumns_algorithmQR(A,numberOfColumns,tol)
    %
    % [indexesColumns,Aselection]=findLinearIndipendentColumns_algorithmQR(A,numberOfColumns,tol)
    %
    %     DESCRIPTION: the function identifies a possible set of linear
    %     indipedent columns of the input matrix A. The algorithm could receives
    %     the number of columns to be extracted as input or it could be
    %     estimated with rankQR(A).
    %
    %     USAGE: in a matlab script or function
    %
    %     INPUT:  - A   = generic matrix
    %             - (numberOfColumns)  = number of columns to be extracted.
    %               If unkwon set numberOfColumns=[]
    %               This input could be omitted
    %             - (tol) = threshold for identifying if the diagonal elements are
    %               different from zeros.
    %               This input could be omitted
    %
    %     OUTPUT: - indexesColumns  = indexes of a possible set of linear
    %               indipedent columns of the input matrix A.
    %             - (Aselection) = subset of A matrix with the selected columns
    %               This output could be omitted
    %
    %     Genoa, July 2020
    %
    %     See also QR, RANKQR.

    if nargin == 1
        numberOfColumns = [];
        tol=1e-3;
    elseif nargin == 2 && isempty(numberOfColumns)
        tol=1e-3;
    end

    [~,R,P] = qr(A,0);

    if isempty(numberOfColumns)
        % rankA = rankQR(A)
        diagValues = abs(diag(R));
        diagValuesNormalized = diagValues/max(diagValues);
        rankA = sum(diagValuesNormalized > tol*max(size(A)));
        numberOfColumns = rankA;
    else
        if numberOfColumns > min(size(A))
            numberOfColumns = min(size(A));
        end
    end

    indexesColumns = sort(P(1:numberOfColumns));

    if nargout > 1
        Aselection = A(:,indexesColumns);
        varargout{1} = Aselection;
    end

end
