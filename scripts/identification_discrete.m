load('matlab_data/robot_data.mat');

angle_shift = 95;
theta = (theta(:) + angle_shift);
theta_dot = theta_dot(:);
pwm = pwm(:);

dt = 0.002;
time = (0:length(theta)-1) * dt;

theta_ddot = [0; diff(theta_dot) / dt];

data = iddata(theta, pwm, dt);

sys_tf = tfest(data, 2, 0, 'Ts', dt);

disp('Estimated Discrete Transfer Function Model:');
disp(sys_tf);

[num, den] = tfdata(sys_tf, 'v');

sys = tf(num, den, dt)

figure;
compare(data, sys_tf);
title('Discrete Model Fit Comparison');