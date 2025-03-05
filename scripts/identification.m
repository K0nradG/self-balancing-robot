load('matlab_data/robot_data.mat');

theta = theta(:);
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