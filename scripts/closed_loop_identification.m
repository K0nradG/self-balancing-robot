load('matlab_data/robot_data.mat');

angle_shift = 95;
theta = (theta(:) + angle_shift);
theta_dot = theta_dot(:);
pwm = pwm(:);

dt = 0.002;
time = (0:length(theta)-1) * dt;

theta_ddot = [0; diff(theta_dot) / dt];

data = iddata(theta, pwm, dt);
sys_tf = tfest(data, 2, 0); % No poles, two zeroes

disp('Estimated Transfer Function Model:');
disp(sys_tf);

num = sys_tf.Numerator;
den = sys_tf.den;

sys = tf(num, den)

figure;
compare(data, sys_tf);
title('Identified Model Fit Comparison');

Kp_a1 = num(1);
a2 = den(2);
a3_minus_Kp_kl = den(3);

Kp = 10.0;

a1 = Kp_a1 / Kp;
a3 = a3_minus_Kp_kl + Kp * a1;

model = tf([0, 0, a1], [1, -a2, -a3])

Ts = 0.002;

[Ac, Bc, Cc, Dc] = tf2ss(model.Numerator{1}, model.Denominator{1});

A_d = eye(size(Ac)) + Ts * Ac;
B_d = Ts * Bc;
C_d = Cc;
D_d = Dc;

model_d = ss(A_d, B_d, C_d, D_d, Ts);
sysd = tf(model_d)
