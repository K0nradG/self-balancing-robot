% Copyright 2026 Filip Dymczyk and Konrad Grucel

function dx = rhs(~, x, u, params)
    a = params(1);
    b = params(2);
    c = params(3);

    theta = x(1);
    theta_dot = x(2);

    theta_dot_dot = -a * theta_dot  + b * theta + c * u;
    dx = [theta_dot; theta_dot_dot];
end

