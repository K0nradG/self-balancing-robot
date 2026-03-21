% Copyright 2026 Filip Dymczyk and Konrad Grucel

clc; clearvars; close
init;

load('merged_robot_data_k1200.0_s-8.5.mat');

theta = theta + deg2rad(angle_shift); % To check if theta is by default in radians and needs shifting.
pwm = pwm * -1; % May need inverting to account for improper signs of angle/pwm.
pwm = pwm / 100; % Scale PWM to be from -1 to 1.

% Optimization inputs:
u = pwm * Torque_max;
xr = theta;
tf_ = (1:length(u)) * Ts;
x0 = [xr(1); 0];

% Initial guess for parameters [a, b, c]:
initial_guess = [0.1, 0.1, 0.1]; % For Torque scaling start like this, or select some other point.

% Run optimization:
options = optimset('MaxFunEvals', 5000, 'MaxIter', 5000);
best_params = fminsearch(@(params) objective_function(params, x0, u, tf_(end), xr), initial_guess, options);

% Display results:
disp('Optimized parameters:');
disp(best_params);
[t,x] = rk4(x0,u,tf_(end), best_params);

figure;
plot(t(2:end), xr);
hold on;
plot(t, x(:,1));
xlim([0, tf_(end)]);
title('Obiekt rzeczywisty i symulacja kąta');
legend('Obiekt rzeczywisty', 'Symulacja', 'Location', 'best');
grid on;

b0 = best_params(3);
b1 = best_params(2);
a2_reg = best_params(1);

% Calculating model parameters based on identification results:
Kp = 1200.0;

a1 = b0 / Kp
a2 = a2_reg
a3 = b1 - b0

disp('Object Continous Transfer Function:');
tf_continous = tf([0, 0, a1], [1, a2, -a3])

% Regular state space representation:
[A, B, C, D] = tf2ss(tf_continous.Numerator{1}, tf_continous.Denominator{1});
ss_continous = ss(A, B, C, D);

ss_object_discrete = c2d(ss(ss_continous.A, ss_continous.B, ss_continous.C, ss_continous.D), Ts, 'zoh');