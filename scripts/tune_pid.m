t = 0:0.01:10;
u = ones(size(t));

[y, ~] = lsim(sys, u, t);

[Kp, Ki, Kd] = tune(sys, t, u, y);

disp(['Optimal PID params:']);
disp(['Kp = ', num2str(Kp)]);
disp(['Ki = ', num2str(Ki)]);
disp(['Kd = ', num2str(Kd)]);

controller = pid(Kp, Ki, Kd);
closed_loop_system = feedback(controller * sys, 1);

figure;
step(closed_loop_system);
title('Closed loop response with optimal PID params');

function controller = pid_controller(Kp, Ki, Kd, s)
controller = Kp + Ki / s + Kd * s;
end

function cost = cost_function(pid_gains, system, t, u, y_ref)

Kp = pid_gains(1);
Ki = pid_gains(2);
Kd = pid_gains(3);

s = tf('s');
controller = pid_controller(Kp, Ki, Kd, s);
closed_loop = feedback(controller * system, 1);

[y_out, ~] = lsim(closed_loop, u, t);

error = y_ref - y_out;
cost = sum(error.^2);
end

function [Kp, Ki, Kd] = tune(system, t, u, y_ref, initial_guess)
if nargin < 5
    initial_guess = [0.01, 0.01, 0.01];
end

lb = [0, 0, 0];
ub = [100, 100, 100];

options = optimset('fmincon');
options = optimset(options, 'MaxFunEvals', 500, 'MaxIter', 500, 'Display', 'iter');

[pid_gains, ~] = fmincon(@(pid_gains) cost_function(pid_gains, system, t, u, y_ref), ...
    initial_guess, [], [], [], [], lb, ub, [], options);

Kp = pid_gains(1);
Ki = pid_gains(2);
Kd = pid_gains(3);
end