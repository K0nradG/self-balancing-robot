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
title('Model Fit Comparison');