% Copyright 2026 Filip Dymczyk and Konrad Grucel

function [t, x] = rk4(x0, u, Ts, params)

    N = length(u);

    % N measurements require N-1 integration intervals
    t = (0:N-1)' * Ts;

    x = zeros(N, length(x0));
    x(1,:) = x0(:)';

    for i = 1:N-1

        h = Ts;
        xi = x(i,:)';

        % Zero-order hold input
        ui = u(i);

        k1 = rhs(t(i), xi, ui, params);
        k2 = rhs(t(i)+h/2, xi+h*k1/2, ui, params);
        k3 = rhs(t(i)+h/2, xi+h*k2/2, ui, params);
        k4 = rhs(t(i)+h, xi+h*k3, ui, params);

        x(i+1,:) = (xi + h/6 * ...
            (k1 + 2*k2 + 2*k3 + k4))';

    end
end
