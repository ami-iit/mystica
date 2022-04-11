function [signalOut,varargout] = applyBoundedGaussianNoise(signalIn,saturationValue,probSaturationValue)
    %
    % [signalOut,{sigma}] = applyBoundedGaussianNoise(signalIn,saturationValue,probSaturationValue)
    %
    %     DESCRIPTION: apply gaussian noise to an input signal
    %
    %     USAGE: in a matlab script or function
    %
    %     INPUT:  - signalIn {matrix}
    %               input signal
    %             - saturationValue {scalar}
    %               maximum value that can be generated. All the values higher
    %               than this will be saturated.
    %             - probSaturationValue {scalar}
    %               probability of obtaining the saturation value
    %
    %     OUTPUT: - signalOut {matrix}
    %               output signal
    %             - [sigma] {scalar}
    %               variance of the Gaussian noise generated
    %
    %     Genoa, November 2020
    %
    %     See also mystica.utils.createBoundedGaussianNoise.


    mu = 0;

    [noise,sigma] = mystica.utils.createBoundedGaussianNoise(size(signalIn),mu,saturationValue,probSaturationValue);
    signalOut = signalIn .* (1 + noise);

    if nargout == 2
        varargout{1} = sigma;
    end

end
