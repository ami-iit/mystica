function [noise,varargout] = createBoundedGaussianNoise(dim,mu,saturationValue,probSaturationValue)
    %
    % [noise,{sigma}] = createBoundedGaussianNoise(dim,mu,saturationValue,probSaturationValue)
    %
    %     DESCRIPTION: create a matrix of gaussian noise
    %
    %     USAGE: in a matlab script or function
    %
    %     INPUT:  - dim {array}: [numberOfRows,numberOfColumns]
    %               dimension of the output matrix
    %             - mu {scalar}
    %               mean of the gaussian curve
    %             - saturationValue {scalar}
    %               maximum value that can be generated. All the values higher
    %               than this will be saturated.
    %             - probSaturationValue {scalar}
    %               probability of obtaining the saturation value
    %
    %     OUTPUT: - noise {matrix}: [dim]
    %               matrix with gaussian noise
    %             - [sigma] {scalar}
    %               variance of the Gaussian noise generated
    %
    %     Genoa, November 2020
    %
    %     See also norminv.

    if length(dim) == 1
        dim = [dim 1];
    end

    % probLimit = P(N(0,1)>Z)
    % prob      = P(N(0,1)<Z)
    cumProbZ = 1 - probSaturationValue;

    Z = invnorm(cumProbZ);
    sigma = (saturationValue-mu)/Z;

    if nargout == 2
        varargout{1} = sigma;
    end

    noise = mu + sigma * randn(dim);

    % saturation
    noise = mystica.utils.saturateSignal(noise,saturationValue);

end

function x = invnorm(p)
    x = -sqrt(2)*erfcinv(2*p);
end
