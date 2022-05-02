function signalOut = saturateSignal(signalIn,upperLimit,lowerLimit)
    %
    % signalOut = saturateSignal(signalIn,upperLimit,lowerLimit)
    %
    %     DESCRIPTION: saturate a signal
    %
    %     USAGE: in a matlab script or function
    %
    %     INPUT:  - signalIn {matrix}
    %               input signal
    %             - upperLimit {scalar}
    %               upper limit
    %             - [lowerLimit] {scalar}
    %               lower limit
    %
    %     OUTPUT: - signalOut {matrix}
    %               output signal
    %
    %     Genoa, November 2020

    if nargin == 2
        lowerLimit = - upperLimit;
    end

    if upperLimit < lowerLimit
        error('upperLimit < lowerLimit')
    end

    signalOut = signalIn;

    signalOut(signalIn > upperLimit) = upperLimit;
    signalOut(signalIn < lowerLimit) = lowerLimit;

end
