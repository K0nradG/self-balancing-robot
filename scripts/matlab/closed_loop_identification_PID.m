clc; clearvars; close
load('matlab_data/k650.3i4.1d2s-98.mat');

% Parameters:
angle_shift = 98;
Ts = 0.001;

% Identification inputs:
theta = theta + deg2rad(angle_shift); % To check if theta is by default in radians.
pwm = pwm * -1; % May need inverting to account for improper signs of angle/pwm.

% Identification:
data = iddata(theta', pwm', Ts);
sys_closed_loop = tfest(data, 3, 2); % No poles, two zeroes

num_closed = sys_closed_loop.Numerator;
den_closed = sys_closed_loop.Denominator;

disp('Estimated Closed Loop Transfer Function:');
tf(num_closed, den_closed)

figure;
compare(data, sys_closed_loop);
title('Identified Model Fit Comparison');

% Calculating model parameters based on identification results:
Kp = 650.3;
Ki = 4.1;
Kd = 2;

b2 = num_closed(1);
b1 = num_closed(2);
b0 = num_closed(3);
c2 = den_closed(2);
c1 = den_closed(3);
c0 = den_closed(4);

a1 = ((b2 / Kd) + (b1 / Kp) + (b0 / Ki)) / 3
a2 = c2 - b2
a3 = c1 - b1

disp('Object Continous Transfer Function:');
tf_continous = tf([0, 0, a1], [1, a2, -a3])

ss_continous = canon(tf_continous);

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
ss_object_discrete = c2d(ss(ss_continous.A, ss_continous.B, ss_continous.C, ss_continous.D), Ts, 'zoh')