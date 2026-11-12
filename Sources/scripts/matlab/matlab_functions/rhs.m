% Copyright 2026 Filip Dymczyk and Konrad Grucel

function dx = rhs(~, x, u_val, params)

    % Physical constants
    g   = 9.81;
    R_m = 12.0;
    K_t = 0.0122;
    K_e = 0.0122;

    r   = 0.033;
    m_w = 0.050;
    I_w = 0.5 * m_w * r^2;

    % Identified parameters
    m   = params(1);
    l   = params(2);
    I_b = params(3);
    b_w = params(4);
    b_p = params(5);
    n   = params(6);

    % State variables
    p      = x(1);
    p_dot  = x(2);
    theta  = x(3);
    th_dot = x(4);

    % Equivalent mass matrix
    M = m + 2*m_w + 2*I_w/r^2;

    D11 = M;
    D12 = -m*l*cos(theta);
    D21 = D12;
    D22 = I_b + m*l^2;

     D = [D11 D12;
     D21 D22];

    % Gravitational and centrifugal terms
    Q_mechanical = [
    -m*l*th_dot^2*sin(theta);
    -m*g*l*sin(theta)
    ];

    omega_motor = n * p_dot / r;

    tau_motor = 2*n*K_t/R_m * ...
                (u_val - K_e*omega_motor);

    % Generalized motor forces
    Q_motor = [tau_motor/r;
          -tau_motor];

    % Viscous friction
    Q_friction = [
        -b_w*p_dot;
        -b_p*th_dot
    ];

    % Total generalized force
    Q = Q_mechanical + Q_motor + Q_friction;

    % Accelerations
    ddq = D \ Q;

    p_ddot  = ddq(1);
    th_ddot = ddq(2);

    % State derivatives
    dx = [
        p_dot;
        p_ddot;
        th_dot;
        th_ddot
    ];

end
