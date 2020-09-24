function[points] = change_of_sign(u, threshold)
    start = 1;
    stop = 1;
    iszero = u(1)<threshold & u(1)>-threshold;
    points = [];
    semilenght = 0
    for j=1:(length(u)-1)
        % strting segment
        if (u(j) < threshold & u(j) > -threshold) & (u(j+1) > threshold | u(j+1) < -threshold)
            start = j+1;
            iszero = false;
        %stopping segment
        elseif (u(j) > threshold | u(j) < -threshold) & (u(j+1) < threshold & u(j+1) > -threshold)
            stop = j;
            points = [points;
                       start stop;];
            iszero = true;
        end
    end
    if ~iszero
        points = [points;
                  start length(u);];
    end
end