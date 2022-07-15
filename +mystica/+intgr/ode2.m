function [tout,yout] = ode2(F,T,y0,opts)

    t0 = T(1);
    tf = T(end);
    if isempty(opts.MaxStep)
        h = (tf-t0)/10;
    else
        h = (tf-t0)/round((tf-t0)/opts.MaxStep);
    end
    time = t0 : h : tf;
    y = y0(:)';
    yout = zeros(length(time),length(y));
    tout = zeros(length(time),1);

    yout(1,:) = y;
    tout(1,:) = t0;

    for i = 2 : length(time)
        t = time(i);
        s1 = F(t,y);             s1 = s1(:)';
        s2 = F(t+h/2, y+h*s1/2); s2 = s2(:)';
        y = y + h*s2;
        yout(i,:) = y;
        tout(i,:) = t;
    end
end
