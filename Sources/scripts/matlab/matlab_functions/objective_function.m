% Copyright 2026 Filip Dymczyk and Konrad Grucel

function residual = objective_function(params, x0, u, Ts, xr)

    [~, x_sim] = rk4(x0, u, Ts, params);

    % Directly measured states
    measured = xr(:, [1 3]);

    % Simulated states
    simulated = x_sim(:, [1 3]);

    % Normalize
    scales = std(measured, 0, 1);
    scales(scales < 1e-6) = 1e-6;

    error_matrix = (measured - simulated) ./ scales;

    residual = error_matrix(:);

end