load('matlab_data/fix-gyro-bed-not-help_s-98_k11.1.mat');

% Parameters:
angle_shift = 98;
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

Kp_a1 = num_closed(1);
a3_minus_Kp_a1 = den_closed(3);

a1 = Kp_a1 / Kp;
a2 = den_closed(2);
a3 = a3_minus_Kp_a1 + Kp_a1;

disp('Object Continous Transfer Function:');
tf_continous = tf([0, 0, a1], [1, a2, -a3])

[Ac, Bc, Cc, Dc] = tf2ss(tf_continous.Numerator{1}, tf_continous.Denominator{1});

% Discretization using Forward-Euler method:
A_d = eye(size(Ac)) + Ts * Ac;
B_d = Ts * Bc;
C_d = Cc;
D_d = Dc;

ss_object_discrete = ss(A_d, B_d, C_d, D_d, Ts);
tf_object_discrete = tf(ss_object_discrete)
