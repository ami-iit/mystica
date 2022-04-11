function [indexesRows,varargout]=findLinearIndipendentRows_algorithmQR(A,numberOfRows,tol)
    %
    % [indexesRows,Aselection]=findLinearIndipendentRows_algorithmQR(A,numberOfRows,tol)
    %
    %     DESCRIPTION: the function identifies a possible set of linear
    %     indipedent rows of the input matrix A. The algorithm could receives
    %     the number of rows to be extracted as input or it could be
    %     estimated with rankQR(A).
    %
    %     USAGE: in a matlab script or function
    %
    %     INPUT:  - A   = generic matrix
    %             - (numberOfRows)  = number of rows to be extracted.
    %               If unkwon set numberOfRows=[]
    %               This input could be omitted
    %             - (tol) = threshold for identifying if the diagonal elements are
    %               different from zeros.
    %               This input could be omitted
    %
    %     OUTPUT: - indexesRows  = indexes of a possible set of linear
    %               indipedent rows of the input matrix A.
    %             - (Aselection) = subset of A matrix with the selected rows
    %               This output could be omitted
    %
    %     Genoa, July 2020
    %
    %     See also QR, RANKQR, mystica.utils.findLinearIndipendentColumns_algorithmQR.

    Atr = transpose(A);

    if nargin == 1
        indexesRows = mystica.utils.findLinearIndipendentColumns_algorithmQR(Atr);
    elseif nargin == 2
        indexesRows = mystica.utils.findLinearIndipendentColumns_algorithmQR(Atr,numberOfRows);
    else
        indexesRows = mystica.utils.findLinearIndipendentColumns_algorithmQR(Atr,numberOfRows,tol);
    end

    if nargout > 1
        Aselection = A(indexesRows,:);
        varargout{1} = Aselection;
    end

end
