function y = smoothSign(x,input)
    arguments
        x
        input.method         char {mustBeMember(input.method,{'sign','sigmoid','atan'})} = 'sign';
        input.settlingTime   double = 1;
        input.toleranceBands double = 0.02;
    end
    h = 1-input.toleranceBands;
    switch input.method
        case 'sign'
            y = sign(x);
        case 'sigmoid'
            ts = -log((1-h)/(1+h)); % ts is the settling time of the unitary sigmoid function
            y = (1./(1+exp(-x*ts/input.settlingTime))-0.5)*2;
        case 'atan'
            ts = tan(h*pi/2); % ts is the settling time of the unitary atan function
            y  = atan(x*ts/input.settlingTime)/(pi/2);
    end
end
