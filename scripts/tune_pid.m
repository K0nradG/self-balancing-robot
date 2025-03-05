s = tf('s');
G = (160.1*s + 1.019e04) / (s^2 + 5.114*s + 4.831e-11);

t = 0:0.01:10;
u = sin(t);
y = sin(t);

[Kp, Ki, Kd] = tune(G, t, u, y);

disp(['Optimal PID params:']);
disp(['Kp = ', num2str(Kp)]);
disp(['Ki = ', num2str(Ki)]);
disp(['Kd = ', num2str(Kd)]);

controller = pid(Kp, Ki, Kd);
closed_loop_system = feedback(controller * G, 1);

figure;
step(closed_loop_system);
title('Closed loop response with optimal PID params');

function controller = pid_controller(Kp, Ki, Kd, s)
controller = Kp + Ki / s + Kd * s;
end

function cost = cost_function(pid_gains, system, t, u, y)

Kp = pid_gains(1);
Ki = pid_gains(2);
Kd = pid_gains(3);

s = tf('s');

controller = pid_controller(Kp, Ki, Kd, s);
closed_loop = feedback(controller * system, 1);

[y_out, t_out] = lsim(closed_loop, u, t);

error = y - interp1(t_out, y_out, t, 'linear', 'extrap');
cost = sum(error.^2);
end

function [Kp, Ki, Kd] = tune(system, t, u, y, initial_guess)
if nargin < 5
    initial_guess = [0.28239543504953135, 9.181766682296, 0.0];
end

lb = [0, 0, 0];
ub = [100, 100, 100];

options = optimset('fminunc');
options = optimset(options, 'MaxFunEvals', 500, 'MaxIter', 500);

[pid_gains, ~] = fminunc(@(pid_gains) cost_function(pid_gains, system, t, u, y), initial_guess, options);

Kp = pid_gains(1);
Ki = pid_gains(2);
Kd = pid_gains(3);
end
