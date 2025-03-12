load('matlab_data/fix-gyro-floor-help_s-98_k11.1.mat');

% Parameters:
angle_shift = deg2rad(98);
Ts = 0.001;

% Identification inputs:
theta = deg2rad(theta + angle_shift);
pwm = pwm * -1;

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
Kp = 11.1;

b0 = num_closed(1);
b1 = den_closed(3);

a1 = b0 / Kp
a2 = den_closed(2)
a3 = b1 - b0

disp('Object Continous Transfer Function:');
tf_continous = tf([0, 0, a1], [1, a2, a3]) % Maybe later it can be tested with -a3 (if the model is right).

% Obtaining observable state-space representation (C = [1, 0]) from a
% controllable one:
ss_controllable = canon(tf_continous, 'companion', 'col');
Ac = ss_controllable.A';
Bc = ss_controllable.C';
Cc = ss_controllable.B';
Dc = ss_controllable.D';

% Discretization using Forward-Euler method:
A_d = eye(size(Ac)) + Ts * Ac;
B_d = Ts * Bc;
C_d = Cc;
D_d = Dc;

ss_object_discrete = ss(A_d, B_d, C_d, D_d, Ts)