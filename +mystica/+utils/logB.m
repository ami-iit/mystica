function y = logB(input)
    arguments
        input.x
        input.base (1,1) double
    end
    y = log(input.x) / log(input.base);
end
