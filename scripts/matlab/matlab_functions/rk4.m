function [t, x] = rk4(x0, u , time, theta)
    % Steps:
    timesteps = length(u);
    h = time / timesteps;
    h_2 = h / 2;
    h_6 = h / 6; 
    h_26 = 2 * h_6;

    % Variables init:
    state_variables = length(x0);
    x = zeros(timesteps + 1, state_variables); 
    t = zeros(timesteps + 1, 1);
    xtmp = x0;
    x(1,:) = x0';
    tt = 0;
    
    % Proper RK4:
    for i=1:timesteps
        dx1 = rhs(tt, xtmp, u(i), theta); 
        tmp = xtmp + h_2 * dx1;
        tt = tt + h_2;

        dx2 = rhs(tt, tmp, u(i), theta);
        tmp = xtmp + h_2 * dx2;

        dx3 = rhs(tt, tmp, u(i), theta);
        tmp = xtmp + h * dx3;
        tt = tt + h_2;

        dx4 = rhs(tt, tmp, u(i), theta);
        xtmp = xtmp + h_6 * (dx1 + dx4) + h_26 * (dx2 + dx3);
        x(i + 1, :) = xtmp';
        t(i + 1) = tt;
    end