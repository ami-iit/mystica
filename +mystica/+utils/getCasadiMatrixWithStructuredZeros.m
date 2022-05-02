function A = getCasadiMatrixWithStructuredZeros(A,input)
    arguments
        A
        input.casadiType char = 'SX'
    end
    
    switch input.casadiType
        case 'SX'
            A(find(full(casadi.DM(A==0))==1)) = casadi.SX(1,1);
        case 'MX'
            A(find(full(casadi.DM(A==0))==1)) = casadi.MX(1,1);
    end
    
end
