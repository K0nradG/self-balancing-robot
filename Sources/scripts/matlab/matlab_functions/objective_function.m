% Copyright 2026 Filip Dymczyk and Konrad Grucel

function error = objective_function(params, x0, u, tf_, xr)
    [~, x] = rk4(x0, u, tf_, params);
    error = sum((xr' - x(1:end-1,1)).^2);
end
