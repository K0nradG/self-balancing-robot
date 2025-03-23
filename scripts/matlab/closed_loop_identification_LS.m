clc; clearvars; close
init

% Adjust input signals
theta = theta + deg2rad(angle_shift); % To check if theta is by default in radians.
pwm = pwm * -1; % May need inverting to account for improper signs of angle/pwm.

% Design a low-pass filter (e.g., a Butterworth filter)
fc = 8;  % Cut-off frequency (in Hz)
[b, a] = butter(2, fc * Ts, 'low');  % Second-order Butterworth filter

% Apply the filter to the theta_dot signal:
theta_dot_filtered = filter(b, a, theta_dot);

% Compute theta_ddot using the filtered theta_dot signal:
theta_ddot = diff(theta_dot_filtered) / Ts;

% Adjust input data:
theta = theta(2:end)';  
theta_dot = theta_dot_filtered(2:end)';
pwm = pwm(2:end)';

% Construct regression matrix:
Phi = [-theta_dot, -theta, pwm];
Y = theta_ddot;

% Regression calc:
Theta = Phi \  Y';

a2_reg = Theta(1);
b1 = Theta(2);
b0 = Theta(3);

% Compute estimated theta_ddot using identified model:
theta_ddot_est = -a2_reg * theta_dot -b1 * theta + b0 * pwm;

% Time vector (must match theta_ddot length)
t = (0:length(theta_ddot)-1) * Ts; 

% Compare estimated vs. measured theta_ddot:
figure;
plot(t, theta_ddot, 'b--', 'DisplayName', 'Measured θ̈'); % Measured acceleration
hold on;
plot(t, theta_ddot_est, 'r', 'DisplayName', 'Estimated θ̈'); % Estimated acceleration

legend;
title('Estimated θ̈ from Model vs. Actual Data');
xlabel('Time [s]');
ylabel('Acceleration θ̈ [rad/s²]');
grid on;

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