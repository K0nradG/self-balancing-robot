load('matlab_data/robot_data.mat');

angle_shift = 95;
theta = (theta(:) + angle_shift);
pwm = pwm(:);

dt = 0.002;
time = (0:length(theta)-1) * dt;

theta_dot = [0; diff(theta) / dt];
theta_ddot = [0; diff(theta_dot) / dt];

data = iddata(theta, pwm, dt);
sys_tf = tfest(data, 2, 1);

disp('Estimated Transfer Function Model:');
disp(sys_tf);

num = sys_tf.Numerator;
den = sys_tf.den;

sys = tf(num, den)

figure;
compare(data, sys_tf); % Compare models with actual data
title('Model Fit Comparison');