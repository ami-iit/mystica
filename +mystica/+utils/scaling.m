function S = scaling(A)
    S = diag(1./max(abs(A),[],1));
end
