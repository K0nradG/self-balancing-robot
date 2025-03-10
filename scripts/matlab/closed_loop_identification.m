load('matlab_data/robot_data.mat');

% Parameters:
angle_shift = 95;
Ts = 0.002;

% Identification inputs:
theta = (theta(:) + angle_shift);
theta_dot = theta_dot(:);
pwm = pwm(:);
time = (0:length(theta)-1) * Ts;
theta_ddot = [0; diff(theta_dot) / Ts];

% Identification:
data = iddata(theta, pwm, Ts);
sys_closed_loop = tfest(data, 2, 0); % No poles, two zeroes

num_closed = sys_closed_loop.Numerator;
den_closed = sys_closed_loop.Denominator;

disp('Estimated Closed Loop Transfer Function:');
tf(num_closed, den_closed)

figure;
compare(data, sys_closed_loop);
title('Identified Model Fit Comparison');

% Calculating model parameters based on identification results:
Kp = 10.0;

Kp_a1 = num_closed(1);
a3_minus_Kp_a1 = den_closed(3);

a1 = Kp_a1 / Kp;
a2 = den_closed(2);
a3 = a3_minus_Kp_a1 + Kp_a1;

disp('Object Continous Transfer Function:');
tf_continous = tf([0, 0, a1], [1, -a2, -a3])

[Ac, Bc, Cc, Dc] = tf2ss(tf_continous.Numerator{1}, tf_continous.Denominator{1});

% Discretization using Forward-Euler method:
A_d = eye(size(Ac)) + Ts * Ac;
B_d = Ts * Bc;
C_d = Cc;
D_d = Dc;

ss_object_discrete = ss(A_d, B_d, C_d, D_d, Ts);
tf_object_discrete = tf(ss_object_discrete)
