clc; clearvars; close
init;

load('merged_robot_data_k1200.0_s-8.5.mat');

% Identification inputs:
theta = theta + deg2rad(angle_shift); % To check if theta is by default in radians.
pwm = pwm * -1; % May need inverting to account for improper signs of angle/pwm.

% Identification:
data = iddata(theta', pwm', Ts);
sys_closed_loop = tfest(data, 2, 0); % No poles, two zeroes

num_closed = sys_closed_loop.Numerator;
den_closed = sys_closed_loop.Denominator;

disp('Estimated Closed Loop Transfer Function:');
tf(num_closed, den_closed)

figure;
compare(data, sys_closed_loop);
title('Identified Model Fit Comparison');

% Calculating model parameters based on identification results:
Kp = 1200.0;

b0 = num_closed(1);
b1 = den_closed(3);

a1 = b0 / Kp 
a2 = den_closed(2)
a3 = b1 - b0

disp('Object Continous Transfer Function:');
tf_continous = tf([0, 0, a1], [1, a2, -a3])

% Regular state space representation:
[A, B, C, D] = tf2ss(tf_continous.Numerator{1}, tf_continous.Denominator{1});
ss_continous = ss(A, B, C, D);

ss_object_discrete = c2d(ss(ss_continous.A, ss_continous.B, ss_continous.C, ss_continous.D), Ts, 'zoh')