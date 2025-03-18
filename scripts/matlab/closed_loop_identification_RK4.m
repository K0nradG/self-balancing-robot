clc; clearvars; close
init

% Adjust input signals:
theta = theta + deg2rad(angle_shift); % To check if theta is by default in radians and needs shifting.
pwm = pwm * -1; % May need inverting to account for improper signs of angle/pwm.
pwm = pwm / 100; % Scale PWM to be from -1 to 1.

% Optimization inputs:
from_cm_to_kg = 1 / 100;
Torque_max = 0.8 * from_cm_to_kg; % [kg * m]
u = pwm * Torque_max;
xr = theta;
tf_ = (1:length(u)) * Ts;
x0 = [xr(1); 0];

% [2.8023, 44.0931, 0.0369] % Best params so far for no torque scaling.
% Initial guess for parameters [a, b, c]:
initial_guess = [1, 1, 1]; % For Torque scaling start like this, or select some other point.

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
Kp = 650.3;

a1 = b0 / Kp 
a2 = a2_reg
a3 = b1 - b0

disp('Object Continous Transfer Function:');
tf_continous = tf([0, 0, a1], [1, a2, -a3])

% Regular state space representation:
[A, B, C, D] = tf2ss(tf_continous.Numerator{1}, tf_continous.Denominator{1});
ss_continous = ss(A, B, C, D);

%Ac = ss_controllable.A';
%Bc = ss_controllable.C';
%Cc = ss_controllable.B';
%Dc = ss_controllable.D';

% Discretization using Forward-Euler method:
%A_d = eye(size(Ac)) + Ts * Ac;
%B_d = Ts * Bc;
%C_d = Cc;
%D_d = Dc;

%ss_object_discrete = ss(A_d, B_d, C_d, D_d, Ts)
%ss_object_discrete = c2d(ss(Ac, Bc, Cc, Dc), Ts, 'zoh')
ss_object_discrete = c2d(ss(ss_continous.A, ss_continous.B, ss_continous.C, ss_continous.D), Ts, 'zoh');