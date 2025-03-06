dt = 0.002;
t = 0:dt:10;
u = zeros(size(t));
y0 = 10 * (180 / pi);

[y, ~] = lsim(sys, u, t, y0);

[Kp, Ki, Kd] = tune(sys, t, u, 0, dt);

disp('Optimal Discrete PID params:');
disp(['Kp = ', num2str(Kp)]);
disp(['Ki = ', num2str(Ki)]);
disp(['Kd = ', num2str(Kd)]);

controller = pid(Kp, Ki, Kd, 'Ts', dt);
closed_loop_system = feedback(series(controller, sys), 1);

figure;
step(closed_loop_system);
title('Closed Loop Response with Optimal Discrete PID Params');

function cost = cost_function(pid_gains, system, t, u, y_ref, dt)
Kp = pid_gains(1);
Ki = pid_gains(2);
Kd = pid_gains(3);

controller = pid(Kp, Ki, Kd, 'Ts', dt);
closed_loop = feedback(series(controller, system), 1);

[y_out, ~] = lsim(closed_loop, u, t);

error = y_ref - y_out;
cost = sum(error.^2);
end

function [Kp, Ki, Kd] = tune(system, t, u, y_ref, dt, initial_guess)
if nargin < 6
    initial_guess = [0.001, 0.001, 0.001];
end

lb = [0, 0, 0];
ub = [1000, 1000, 1000];

options = optimset('fmincon');
options = optimset(options, 'MaxFunEvals', 500, 'MaxIter', 500, 'Display', 'iter');

[pid_gains, ~] = fmincon(@(pid_gains) cost_function(pid_gains, system, t, u, y_ref, dt), ...
    initial_guess, [], [], [], [], lb, ub, [], options);

Kp = pid_gains(1);
Ki = pid_gains(2);
Kd = pid_gains(3);
end
